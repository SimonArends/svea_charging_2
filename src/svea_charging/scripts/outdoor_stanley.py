#!/usr/bin/env python3

"""Standalone, fail-safe outdoor Stanley waypoint follower.

This node is deliberately separate from the charging mission. Its route is a
fixed sequence of waypoints in the outdoor ``map`` frame.
"""

import ast
import math
from collections import deque

from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, UInt8
from tf_transformations import euler_from_quaternion

from svea_charging.controllers.stanleyController import StanleyController
from svea_core import rosonic as rx
from svea_core.interfaces import ActuationInterface


class OutdoorStanley(rx.Node):
    update_hz = rx.Parameter(20.0)
    enabled = rx.Parameter(False)
    target_velocity = rx.Parameter(0.20)
    turn_velocity = rx.Parameter(0.12)
    goal_tolerance = rx.Parameter(0.75)
    start_tolerance = rx.Parameter(1.50)
    start_heading_tolerance = rx.Parameter(0.50)
    corridor_width = rx.Parameter(1.50)
    turn_curvature_threshold = rx.Parameter(0.50)
    minimum_turning_radius = rx.Parameter(0.40)
    odometry_timeout_s = rx.Parameter(0.30)
    gps_timeout_s = rx.Parameter(2.50)
    rtk_timeout_s = rx.Parameter(2.50)
    require_rtk_fixed = rx.Parameter(True)
    rtk_fixed_settle_s = rx.Parameter(10.0)
    max_horizontal_accuracy = rx.Parameter(0.50)
    localization_settle_s = rx.Parameter(10.0)
    max_settle_position_spread = rx.Parameter(0.20)

    # Fixed [x, y] positions in the outdoor global EKF's map frame.
    map_waypoints = rx.Parameter(
        "[[0.0, 0.0], [5.0, 0.0], [10.0, 0.0], [20.0, 0.0], "
        "[30.0, 0.0], [40.0, 0.0], [50.0, 0.0]]"
    )
    odometry_topic = rx.Parameter("odometry/global")
    gps_topic = rx.Parameter("gps/fix")
    carrier_solution_topic = rx.Parameter("gps/carrier_solution")
    horizontal_accuracy_topic = rx.Parameter("gps/horizontal_accuracy")

    actuation = ActuationInterface()

    @rx.Subscriber(Odometry, odometry_topic)
    def _odometry_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.state = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(yaw),
            float(msg.twist.twist.linear.x),
        )
        self.last_odom_s = self._now_s()
        samples = getattr(self, "settle_samples", None)
        if samples is not None and not getattr(self, "path_ready", False):
            samples.append((self.last_odom_s, self.state[0], self.state[1]))

    @rx.Subscriber(NavSatFix, gps_topic, qos_profile=qos_profile_sensor_data)
    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self.last_gps_s = self._now_s()

    @rx.Subscriber(UInt8, carrier_solution_topic)
    def _carrier_solution_cb(self, msg: UInt8):
        now = self._now_s()
        new_solution = int(msg.data)
        if new_solution == 2:
            if self.carrier_solution != 2:
                self.rtk_fixed_since = now
        else:
            self.rtk_fixed_since = None
        self.carrier_solution = new_solution
        self.last_rtk_s = now

    @rx.Subscriber(Float64, horizontal_accuracy_topic)
    def _horizontal_accuracy_cb(self, msg: Float64):
        self.horizontal_accuracy = float(msg.data)
        self.last_accuracy_s = self._now_s()

    def on_startup(self):
        self.state = None
        self.last_odom_s = None
        self.last_gps_s = None
        self.last_rtk_s = None
        self.carrier_solution = None
        self.rtk_fixed_since = None
        self.horizontal_accuracy = None
        self.last_accuracy_s = None
        self.settle_samples = deque(maxlen=2000)
        self.path_ready = False
        self.finished = False
        self.route_error = None
        self.stop_reason = None
        self.was_enabled = self._enabled_now()
        self.controller = StanleyController(node=self)
        self.controller.target_velocity = float(self.target_velocity)
        self.waypoints = self._parse_waypoints(str(self.map_waypoints))
        self.start = self.waypoints[0]
        self.goal = self.waypoints[-1]
        self.route_heading = math.atan2(
            self.waypoints[1][1] - self.start[1],
            self.waypoints[1][0] - self.start[0],
        )

        period = 1.0 / max(float(self.update_hz), 1.0)
        self.create_timer(period, self.loop)
        self.get_logger().warn(
            "Outdoor Stanley is DISABLED" if not self.was_enabled
            else "Outdoor Stanley is ENABLED; waiting for fresh GPS and global odometry"
        )

    @staticmethod
    def _parse_waypoints(value):
        points = ast.literal_eval(value)
        if not isinstance(points, (list, tuple)) or len(points) < 2:
            raise ValueError("map_waypoints must contain at least two [x, y] points")
        parsed = []
        for point in points:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError("each map waypoint must be [x, y]")
            parsed.append([float(point[0]), float(point[1])])
        return parsed

    def _initialize_path(self):
        # update_traj prepends the live state. Skip the configured start point
        # here to avoid a zero-length spline segment when the poses coincide.
        self.controller.update_traj(self.state, self.waypoints[1:])
        peak_curvature = max(abs(value) for value in self.controller.ck)
        curvature_limit = 1.0 / float(self.minimum_turning_radius)
        if peak_curvature > curvature_limit:
            self.route_error = (
                f"route curvature {peak_curvature:.2f} 1/m exceeds "
                f"limit {curvature_limit:.2f} 1/m"
            )
            return
        self.path_ready = True
        self.get_logger().info(
            f"Outdoor map route initialized with {len(self.waypoints)} waypoints; "
            f"goal=({self.goal[0]:.2f}, {self.goal[1]:.2f}) in map"
        )

    def loop(self):
        # A disabled observer must not compete with manual control or another
        # controller for the LLI. Once enabled, every inhibit condition sends
        # an explicit stop command.
        enabled = self._enabled_now()
        if not enabled:
            # Do not publish while the node starts disabled, so it cannot
            # compete with manual control. If it is disabled at runtime after
            # having been active, send one explicit stop before going silent.
            if self.was_enabled:
                self.actuation.send_control(0.0, 0.0)
            if self.stop_reason != "disabled":
                self.get_logger().warn("Outdoor Stanley inactive: disabled")
                self.stop_reason = "disabled"
            self.was_enabled = False
            return
        if not self.was_enabled:
            self.get_logger().warn(
                "Outdoor Stanley enabled at runtime; applying safety checks"
            )
        self.was_enabled = True

        reason = self._inhibit_reason()
        if reason is not None:
            self._stop(reason)
            return

        self.stop_reason = None
        distance = math.hypot(self.goal[0] - self.state[0], self.goal[1] - self.state[1])
        at_path_end = self.controller.target_idx >= len(self.controller.cx) - 2
        if distance <= float(self.goal_tolerance) and at_path_end:
            self.finished = True
            self._stop("goal reached")
            return

        self._set_speed_for_curvature()
        steering, velocity = self.controller.compute_control(self.state)
        self.actuation.send_control(float(steering), -float(velocity))

    def _inhibit_reason(self):
        if self.finished:
            return "goal reached"
        if self.state is None:
            return "waiting for global odometry"
        now = self._now_s()
        if self.last_odom_s is None or now - self.last_odom_s > float(self.odometry_timeout_s):
            return "global odometry stale"
        if self.last_gps_s is None or now - self.last_gps_s > float(self.gps_timeout_s):
            return "GPS fix stale or unavailable"
        if bool(self.require_rtk_fixed):
            if self.last_rtk_s is None or now - self.last_rtk_s > float(self.rtk_timeout_s):
                return "RTK status stale or unavailable"
            if self.carrier_solution != 2:
                return f"RTK is not fixed (carrSoln={self.carrier_solution})"
            if (
                self.rtk_fixed_since is None
                or now - self.rtk_fixed_since < float(self.rtk_fixed_settle_s)
            ):
                return "waiting for RTK fixed to stabilize"
        if self.last_accuracy_s is None or now - self.last_accuracy_s > float(self.gps_timeout_s):
            return "GPS horizontal accuracy stale or unavailable"
        if self.horizontal_accuracy > float(self.max_horizontal_accuracy):
            return (
                f"GPS horizontal accuracy too poor "
                f"({self.horizontal_accuracy:.2f} m)"
            )
        if self.route_error is not None:
            return self.route_error
        if not self.path_ready:
            settle_reason = self._settling_reason(now)
            if settle_reason is not None:
                return settle_reason
            start_distance = math.hypot(
                self.state[0] - self.start[0], self.state[1] - self.start[1]
            )
            if start_distance > float(self.start_tolerance):
                return f"outside start tolerance ({start_distance:.2f} m)"
            heading_error = abs(self._normalize_angle(self.state[2] - self.route_heading))
            if heading_error > float(self.start_heading_tolerance):
                return f"outside start heading tolerance ({heading_error:.2f} rad)"
            self._initialize_path()
            if not self.path_ready:
                return self.route_error or "route initialization failed"
        cross_track_distance = self._distance_to_trajectory()
        if cross_track_distance > float(self.corridor_width):
            return f"outside path corridor ({cross_track_distance:.2f} m)"
        return None

    def _settling_reason(self, now):
        required_duration = float(self.localization_settle_s)
        cutoff = now - required_duration
        while self.settle_samples and self.settle_samples[0][0] < cutoff:
            self.settle_samples.popleft()
        if not self.settle_samples or now - self.settle_samples[0][0] < required_duration * 0.95:
            return "waiting for localization to settle"
        xs = [sample[1] for sample in self.settle_samples]
        ys = [sample[2] for sample in self.settle_samples]
        spread = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        if spread > float(self.max_settle_position_spread):
            return f"localization not stable ({spread:.2f} m spread)"
        return None

    def _distance_to_trajectory(self):
        x, y = self.state[0], self.state[1]
        return min(math.hypot(x - px, y - py) for px, py in zip(
            self.controller.cx, self.controller.cy
        ))

    def _set_speed_for_curvature(self):
        start = self.controller.target_idx
        end = min(start + 21, len(self.controller.ck))
        upcoming = self.controller.ck[start:end]
        upcoming_curvature = max((abs(value) for value in upcoming), default=0.0)
        if upcoming_curvature >= float(self.turn_curvature_threshold):
            self.controller.target_velocity = float(self.turn_velocity)
        else:
            self.controller.target_velocity = float(self.target_velocity)

    @staticmethod
    def _normalize_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def _stop(self, reason):
        self.actuation.send_control(0.0, 0.0)
        if reason != self.stop_reason:
            log = self.get_logger().info if reason == "goal reached" else self.get_logger().warn
            log(f"Outdoor Stanley stopped: {reason}")
            self.stop_reason = reason

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _enabled_now(self):
        """Read the live ROS value; rx.Parameter caches its startup value."""
        return bool(self.get_parameter("enabled").value)


if __name__ == "__main__":
    OutdoorStanley.main()
