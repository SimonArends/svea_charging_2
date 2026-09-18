#!/usr/bin/env python3

"""Fail-safe outdoor Stanley waypoint follower.

Its route is a fixed sequence of waypoints in the outdoor ``map`` frame.
It publishes Stanley command topics so the charging BT/control_mux is the
only thing that owns actuation.

This node shuttles indefinitely between two endpoints, "A" and "B".
``map_waypoints`` describes the A -> B leg (first point is A, last point
is B) and ``map_waypoints_b_to_a`` independently describes the B -> A leg
(first point is B, last point is A) -- the two routes need not be the
same path. On every goal arrival the node re-runs the full startup
sequence (localization settle, start-position tolerance, start-heading
tolerance, path initialization) before departing on the opposite leg,
exactly as it does on a cold start.
"""

import ast
import math
from collections import deque

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32, Float64, String, UInt8
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

from svea_charging.controllers.stanleyController import StanleyController
from svea_core import rosonic as rx
#from svea_core.interfaces import LocalizationInterface

class OutdoorStanley(rx.Node):
    update_hz = rx.Parameter(20.0)
    enabled = rx.Parameter(False)
    target_velocity = rx.Parameter(0.28)
    turn_velocity = rx.Parameter(0.12)
    max_steering_rad = rx.Parameter(0.45)
    goal_tolerance = rx.Parameter(0.75)
    start_tolerance = rx.Parameter(1.50)
    start_heading_tolerance = rx.Parameter(0.50)
    corridor_width = rx.Parameter(1.50)
    turn_curvature_threshold = rx.Parameter(0.50)
    minimum_turning_radius = rx.Parameter(0.40)
    odometry_timeout_s = rx.Parameter(0.30)
    use_gps = rx.Parameter(False)
    gps_timeout_s = rx.Parameter(2.50)
    rtk_timeout_s = rx.Parameter(2.50)
    require_rtk_fixed = rx.Parameter(False)
    rtk_fixed_settle_s = rx.Parameter(10.0)
    max_horizontal_accuracy = rx.Parameter(0.50)
    localization_settle_s = rx.Parameter(10.0)
    max_settle_position_spread = rx.Parameter(0.20)
    use_course_heading = rx.Parameter(False)
    course_heading_min_distance = rx.Parameter(0.25)
    course_heading_alpha = rx.Parameter(0.35)
    controller_name = rx.Parameter("transport_stanley")
    active_controller = rx.Parameter("idle")
    #localizer = LocalizationInterface()

    # Which endpoint the node starts at: "A" or "B". "A" starts the A -> B
    # leg; "B" starts the B -> A leg. At runtime this is overwritten with
    # "A", "B", "travelling_to_A" or "travelling_to_B" to reflect live state.
    location = rx.Parameter("A")

    # Fixed [x, y] positions in the outdoor global EKF's map frame.
    # map_waypoints is the A -> B route: first point is A, last is B.
    map_waypoints = rx.Parameter(
        "[[0.0, 0.0], [5.0, 0.0], [10.0, 0.0], [20.0, 0.0], "
        "[30.0, 0.0], [40.0, 0.0], [50.0, 0.0]]"
    )
    # map_waypoints_b_to_a is the independent B -> A route: first point is
    # B, last is A. It does not have to retrace map_waypoints.
    map_waypoints_b_to_a = rx.Parameter(
        "[[50.0, 0.0], [40.0, 0.0], [30.0, 0.0], [20.0, 0.0], "
        "[10.0, 0.0], [5.0, 0.0], [0.0, 0.0]]"
    )
    odometry_topic = rx.Parameter("/svea3/odom")
    gps_topic = rx.Parameter("gps/fix")
    carrier_solution_topic = rx.Parameter("gps/carrier_solution")
    horizontal_accuracy_topic = rx.Parameter("gps/horizontal_accuracy")
    steering_cmd_topic = rx.Parameter("stanley/cmd_steering_rad")
    velocity_cmd_topic = rx.Parameter("stanley/cmd_velocity_mps")

    command_steering_pub = rx.Publisher(Float32, steering_cmd_topic)
    command_velocity_pub = rx.Publisher(Float32, velocity_cmd_topic)
    status_pub = rx.Publisher(String, "outdoor_stanley/status")
    location_pub = rx.Publisher(String, "outdoor_stanley/location")
    course_heading_pub = rx.Publisher(Float64, "outdoor_stanley/course_heading")
    cross_track_error_pub = rx.Publisher(Float64, "outdoor_stanley/cross_track_error")
    yaw_error_pub = rx.Publisher(Float64, "outdoor_stanley/yaw_error")
    steering_cmd_pub = rx.Publisher(Float64, "outdoor_stanley/steering_cmd")
    velocity_cmd_pub = rx.Publisher(Float64, "outdoor_stanley/velocity_cmd")
    target_index_pub = rx.Publisher(UInt8, "outdoor_stanley/target_index")
    goal_pub = rx.Publisher(Marker, "outdoor_stanley/goal_marker")
    waypoints_pub = rx.Publisher(Marker, "outdoor_stanley/waypoints_marker")
    traj_pub = rx.Publisher(Marker, "outdoor_stanley/traj_marker")

    @rx.Subscriber(Odometry, odometry_topic)
    def _odometry_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        odom_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2] - math.pi/2
        odom_yaw = math.atan2(math.sin(odom_yaw), math.cos(odom_yaw))  # wrap to [-pi, pi]
        x = float(msg.pose.pose.position.y)
        y = -float(msg.pose.pose.position.x)
        yaw = self._heading_from_course(x, y, odom_yaw) + 0*1.57
        self.state = (
            x,
            y,
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

    @rx.Subscriber(String, "mission/active_controller")
    def _mission_active_cb(self, msg: String):
        self.active_controller = msg.data

    def on_startup(self):
        #self.state = self.localizer.get_state()
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
        self.was_enabled = self._is_control_active()
        self.last_course_point = None
        self.course_heading = None
        self.viz_counter = 0
        self.controller = StanleyController(node=self)
        self.controller.target_velocity = float(self.target_velocity)

        # map_waypoints (A -> B) and map_waypoints_b_to_a (B -> A) are two
        # independent routes, each parsed once so a leg switch is just a
        # pointer swap, not a re-parse.
        self.route_a_to_b = self._parse_waypoints(str(self.map_waypoints))
        self.route_b_to_a = self._parse_waypoints(str(self.map_waypoints_b_to_a))

        # Trusted directly from the parameter, no validation: "A" starts the
        # A -> B leg, anything else starts the B -> A leg.
        self.location = str(self.location)
        # direction: which endpoint the *active* leg is currently heading
        # toward ("to_A" or "to_B").
        self.direction = "to_B" if self.location == "A" else "to_A"
        self.waypoints = self.route_a_to_b if self.direction == "to_B" else self.route_b_to_a

        self.start = self.waypoints[0]
        self.goal = self.waypoints[-1]
        self.route_heading = math.atan2(
            self.waypoints[1][1] - self.start[1],
            self.waypoints[1][0] - self.start[0],
        )
        if bool(self.use_course_heading):
            self.course_heading = self.route_heading
        self._publish_location()

        period = 1.0 / max(float(self.update_hz), 1.0)
        self.create_timer(period, self.loop)
        self.get_logger().warn(
            "Outdoor Stanley is inactive" if not self.was_enabled
            else "Outdoor Stanley is active; waiting for fresh odometry"
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
        destination = "B" if self.direction == "to_B" else "A"
        self._set_location(f"travelling_to_{destination}")
        self.get_logger().info(
            f"Route toward {destination} initialized with {len(self.waypoints)} "
            f"waypoints; goal=({self.goal[0]:.2f}, {self.goal[1]:.2f}) in map"
        )

    def loop(self):
        # self.state = self.localizer.get_state()
        # self.last_odom_s = self._now_s()
        # samples = getattr(self, "settle_samples", None)
        # if samples is not None and not getattr(self, "path_ready", False):
        #     samples.append((self.last_odom_s, self.state[0], self.state[1]))
        self._publish_location()
        # A disabled observer must not compete with manual control or another
        # controller for the LLI. Once enabled, every inhibit condition sends
        # an explicit stop command.
        enabled = self._is_control_active()
        if not enabled:
            # Do not publish while the node starts disabled, so it cannot
            # compete with manual control. If it is disabled at runtime after
            # having been active, send one explicit stop before going silent.
            if self.was_enabled:
                self._send_control(0.0, 0.0)
            if self.stop_reason != "disabled":
                self._publish_status("idle")
                self.get_logger().warn("Outdoor Stanley inactive")
                self.stop_reason = "disabled"
            self.was_enabled = False
            return
        if not self.was_enabled:
            self.get_logger().warn("Outdoor Stanley activated at runtime; applying safety checks")
            self.controller.reset_pid()
            self.finished = False
            self.path_ready = False          # forces _initialize_path() to re-run
            self.controller.target_idx = 0
            self.route_error = None
            self.settle_samples.clear()      # re-do the localization settle wait
        self.was_enabled = True

        reason = self._inhibit_reason()
        if reason is not None:
            self._stop(reason)
            return

        self.stop_reason = None
        self._publish_viz_if_due()
        distance = math.hypot(self.goal[0] - self.state[0], self.goal[1] - self.state[1])
        at_path_end = self.controller.target_idx >= len(self.controller.cx) - 2
        if distance <= float(self.goal_tolerance) and at_path_end:
            self.finished = True
            self._arrive_and_prepare_next_leg()
            return

        self._set_speed_for_curvature()
        steering, velocity = self.controller.compute_control(self.state)
        steering, velocity = self._limit_command(steering, velocity)
        self._publish_control_debug(steering, velocity)
        self._publish_status("running")
        self._send_control(float(steering), float(velocity))

    def _arrive_and_prepare_next_leg(self):
        # Stop and report "goal_reached" for this leg first, exactly like a
        # single-shot run would, then immediately queue up the return leg.
        self._stop("goal reached")
        arrived_at = "B" if self.direction == "to_B" else "A"
        self._set_location(arrived_at)
        self.get_logger().info(f"Arrived at {arrived_at}; loading the return leg")
        self._load_next_leg()

    def _load_next_leg(self):
        self.direction = "to_A" if self.direction == "to_B" else "to_B"
        self.waypoints = self.route_a_to_b if self.direction == "to_B" else self.route_b_to_a
        self.start = self.waypoints[0]
        self.goal = self.waypoints[-1]
        self.route_heading = math.atan2(
            self.waypoints[1][1] - self.start[1],
            self.waypoints[1][0] - self.start[0],
        )
        if bool(self.use_course_heading):
            # Restart the course-heading filter pointing the new direction;
            # otherwise it carries the old (now ~180 degree wrong) estimate
            # into the first ticks of the new leg.
            self.course_heading = self.route_heading
        self.last_course_point = None

        # Force the full startup sequence to run again: settle, start
        # tolerance, start heading tolerance, then path (re)initialization.
        # self.finished is reset here (not left True) as a normal part of
        # this; if anything ever throws between the caller setting it and
        # this reset, the node fails safe by staying stopped.
        self.finished = False
        self.path_ready = False
        self.route_error = None
        self.controller.target_idx = 0
        self.settle_samples.clear()

    def _inhibit_reason(self):
        if self.finished:
            return "goal reached"
        if self.state is None:
            return "waiting for global odometry"
        now = self._now_s()
        if self.last_odom_s is None or now - self.last_odom_s > float(self.odometry_timeout_s):
            return "global odometry stale"
        if bool(self.use_gps):
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
            if heading_error > float(self.start_heading_tolerance) and not bool(self.use_course_heading):
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

    def _limit_command(self, steering, velocity):
        # Velocity is intentionally not re-clamped to target_velocity here:
        # the PID in StanleyController already converges to target_velocity
        # (and is bounded by its own max_velocity), so clamping on top of it
        # just pins the output and prevents the PID from ever correcting.
        max_steering = abs(float(self.max_steering_rad))
        steering = min(max(float(steering), -max_steering), max_steering)
        return steering, float(velocity)

    @staticmethod
    def _normalize_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def _stop(self, reason):
        self._send_control(0.0, 0.0)
        self._publish_stop_debug()
        # Clear the velocity PID here, not on resume: it may sit stopped for a
        # long time, and resuming should start clean rather than carry over a
        # stale integral that wound up before/during the stop.
        self.controller.reset_pid()
        status = "goal_reached" if reason == "goal reached" else "blocked"
        if "RTK" in reason:
            status = "rtk_lost"
        self._publish_status(status)
        if reason != self.stop_reason:
            #log = self.get_logger().info if reason == "goal reached" else self.get_logger().warn
            self.get_logger().info(f"Outdoor Stanley stopped: {reason}")
            self.stop_reason = reason

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _enabled_now(self):
        """Read the live ROS value; rx.Parameter caches its startup value."""
        return bool(self.get_parameter("enabled").value)

    def _is_control_active(self):
        if not self._enabled_now():
            return False
        return str(self.active_controller) == str(self.controller_name)

    def _send_control(self, steering, velocity):
        self.command_steering_pub.publish(Float32(data=float(steering)))
        self.command_velocity_pub.publish(Float32(data=float(velocity)))

    def _publish_status(self, status):
        self.status_pub.publish(String(data=str(status)))

    def _set_location(self, value):
        self.location = str(value)
        self._publish_location()

    def _publish_location(self):
        self.location_pub.publish(String(data=str(self.location)))

    def _heading_from_course(self, x, y, fallback_yaw):
        if not bool(self.use_course_heading):
            return fallback_yaw

        current = (x, y)
        if self.last_course_point is None:
            self.last_course_point = current
            return self.course_heading if self.course_heading is not None else fallback_yaw

        last_x, last_y = self.last_course_point
        dx = x - last_x
        dy = y - last_y
        distance = math.hypot(dx, dy)
        if distance < float(self.course_heading_min_distance):
            return self.course_heading if self.course_heading is not None else fallback_yaw

        measured_heading = math.atan2(dy, dx)
        if self.course_heading is None:
            self.course_heading = measured_heading
        else:
            alpha = min(max(float(self.course_heading_alpha), 0.0), 1.0)
            correction = self._normalize_angle(measured_heading - self.course_heading)
            self.course_heading = self._normalize_angle(self.course_heading + alpha * correction)
        self.last_course_point = current
        return self.course_heading

    def _publish_control_debug(self, steering, velocity):
        self.steering_cmd_pub.publish(Float64(data=float(steering)))
        self.velocity_cmd_pub.publish(Float64(data=float(velocity)))
        self.cross_track_error_pub.publish(Float64(data=float(self.controller.cross_track_error)))
        self.yaw_error_pub.publish(Float64(data=float(self.controller.yaw_error)))
        if self.course_heading is not None:
            self.course_heading_pub.publish(Float64(data=float(self.course_heading)))
        self.target_index_pub.publish(UInt8(data=min(int(self.controller.target_idx), 255)))

    def _publish_stop_debug(self):
        self.steering_cmd_pub.publish(Float64(data=0.0))
        self.velocity_cmd_pub.publish(Float64(data=0.0))
        if self.course_heading is not None:
            self.course_heading_pub.publish(Float64(data=float(self.course_heading)))

    def _publish_viz_if_due(self):
        # Route/goal are static once the path is built; republish every
        # ~1s (not every tick) so late-joining RViz/Foxglove subscribers
        # still pick them up.
        self.viz_counter += 1
        if self.viz_counter % max(int(self.update_hz), 1) != 0:
            return
        self._publish_goal_marker()
        self._publish_waypoints_marker()
        self._publish_traj_marker()

    def _publish_goal_marker(self):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = "outdoor_stanley_goal"
        msg.id = 0
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD
        msg.pose.position.x = float(self.goal[0])
        msg.pose.position.y = float(self.goal[1])
        msg.pose.position.z = 0.2
        msg.pose.orientation.w = 1.0
        msg.scale.x = 0.4
        msg.scale.y = 0.4
        msg.scale.z = 0.4
        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 1.0
        msg.color.a = 1.0
        self.goal_pub.publish(msg)

    def _publish_waypoints_marker(self):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = "outdoor_stanley_waypoints"
        msg.id = 0
        msg.type = Marker.LINE_STRIP
        msg.action = Marker.ADD
        msg.scale.x = 0.05
        msg.color.r = 1.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        msg.color.a = 1.0
        msg.points = []
        for wp in self.waypoints:
            p = Point()
            p.x = float(wp[0])
            p.y = float(wp[1])
            p.z = 0.05
            msg.points.append(p)
        self.waypoints_pub.publish(msg)

    def _publish_traj_marker(self):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = "outdoor_stanley_traj"
        msg.id = 0
        msg.type = Marker.LINE_STRIP
        msg.action = Marker.ADD
        msg.scale.x = 0.05
        msg.color.r = 0.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        msg.color.a = 1.0
        msg.points = []
        for x, y in zip(self.controller.cx, self.controller.cy):
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.03
            msg.points.append(p)
        self.traj_pub.publish(msg)


if __name__ == "__main__":
    OutdoorStanley.main()
