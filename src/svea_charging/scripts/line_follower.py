#!/usr/bin/env python3

from collections import deque

import cv2
import numpy as np
import tf2_geometry_msgs  # Registers geometry message transforms with tf2.
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

from svea_core import rosonic as rx
from svea_core.interfaces import LocalizationInterface
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)

#QoS Profile
qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class LineFollowerLocalizationInterface(LocalizationInterface):
    def _resolve_base_frame(self, odom=None):
        base_frame = str(self.localization.base_frame)
        if base_frame and base_frame != "self/base_link":
            return base_frame

        namespace = self.node.get_namespace().strip("/")
        if namespace:
            return f"{namespace}/base_link"

        if odom is not None and odom.child_frame_id:
            return odom.child_frame_id

        return base_frame

    def transform_odom(
        self,
        odom,
        pose_target=None,
        twist_target=None,
        timeout_s=0.2,
    ):
        resolved_twist_target = (
            twist_target
            if twist_target is not None
            else self._resolve_base_frame(odom)
        )
        return super().transform_odom(
            odom,
            pose_target=pose_target,
            twist_target=resolved_twist_target,
            timeout_s=timeout_s,
        )


class line_follower(rx.Node):
    dt = rx.Parameter(0.05)
    image_topic = rx.Parameter("/svea67/image_raw")
    target_velocity = rx.Parameter(0.26)
    max_velocity = rx.Parameter(0.4)
    stop_on_lost_line = rx.Parameter(True)
    controller_name = rx.Parameter("line_follower")
    active_controller = rx.Parameter('idle')
    steering_cmd_topic = rx.Parameter("line_follower/cmd_steering_rad")
    velocity_cmd_topic = rx.Parameter("line_follower/cmd_velocity_mps")


    publish_debug_image = rx.Parameter(True)
    debug_image_topic = rx.Parameter("line_follower/debug_image")
    debug_publish_every_n = rx.Parameter(3)

    # Outdoor defaults per LINE_FOLLOWER_CAMERA_HANDOFF.md problem 2: indoor
    # thresholds required S>=100, but the yellow line outdoors measured
    # S~50-90 (H~19-23, V~176-189). Re-derive per-robot before trusting these.
    # lower_h = rx.Parameter(15)
    # lower_s = rx.Parameter(40)
    # lower_v = rx.Parameter(80)
    # upper_h = rx.Parameter(35)
    # upper_s = rx.Parameter(255)
    # upper_v = rx.Parameter(255)

    lower_h = rx.Parameter(20)
    lower_s = rx.Parameter(100)
    lower_v = rx.Parameter(100)
    upper_h = rx.Parameter(35)
    upper_s = rx.Parameter(255)
    upper_v = rx.Parameter(255)


    crop_start_ratio = rx.Parameter(0.55)
    min_contour_area = rx.Parameter(120)
    steering_kp = rx.Parameter(1.5)
    steering_ki = rx.Parameter(.4)
    steering_kd = rx.Parameter(0.0)
    steering_limit_rad = rx.Parameter(0.6)
    lost_line_steering_rad = rx.Parameter(0.0)
    # Constant pixel offset added to the image center used for steering
    # error. Compensates a camera mount that's slightly off the vehicle's
    # true centerline (shows up as a consistent sideways offset while
    # tracking a straight line). See loop().
    steering_bias_px = rx.Parameter(50.0)
    velocity_scale_from_error = rx.Parameter(False)

    use_aruco_stop = rx.Parameter(True)
    aruco_distance_topic = rx.Parameter("aruco/distance_m")
    aruco_stop_distance_m = rx.Parameter(0.622)
    platform_transition_distance_m = rx.Parameter(1.0)
    ramp_min_velocity = rx.Parameter(0.3)
    approach_deceleration_mps2 = rx.Parameter(0.6)
    # Keep correcting until the behaviour tree detects charging. Set this above
    # zero only if a stationary acceptance band is desired.
    dock_tolerance_m = rx.Parameter(0.0)
    aruco_velocity_kp = rx.Parameter(0.35)
    aruco_velocity_ki = rx.Parameter(0.03)
    aruco_velocity_kd = rx.Parameter(0.0)
    aruco_velocity_integral_limit = rx.Parameter(0.3)
    aruco_max_backup_velocity = rx.Parameter(0.3)
    aruco_min_forward_command = rx.Parameter(0.25)
    aruco_min_backup_command = rx.Parameter(0.3)
    # Must exceed dock_settle_time_s to actually take effect — see
    # _change_dock_search_direction(), which waits
    # max(dock_settle_time_s, reverse_neutral_time_s) before reversing.
    # 0.25 < the old dock_settle_time_s default (0.30) was a no-op: the ESC
    # may need a real neutral pause before reverse arms, and the docking
    # bag (line_follower_docking_check) showed commanded reverse averaging
    # -0.257 m/s for 6.75s straight while measured velocity averaged only
    # -0.017 m/s — i.e. reverse was requested but barely happened.
    reverse_neutral_time_s = rx.Parameter(0.6)
    dock_search_half_width_m = rx.Parameter(0.015)
    dock_settle_time_s = rx.Parameter(0.30)
    velocity_command_slew_mps2 = rx.Parameter(0.7)
    aruco_filter_window = rx.Parameter(5)
    aruco_timeout_s = rx.Parameter(0.5)

    localizer = LineFollowerLocalizationInterface()

    steering_cmd_pub = rx.Publisher(Float32, steering_cmd_topic)
    velocity_cmd_pub = rx.Publisher(Float32, velocity_cmd_topic)
    line_error_pub = rx.Publisher(Float32, "line_follower/error_px")
    status_pub = rx.Publisher(String, "line_follower/status")
    centroid_pub = rx.Publisher(Point, "line_follower/centroid")
    debug_image_pub = rx.Publisher(Image, debug_image_topic)
    desired_velocity_pub = rx.Publisher(
        Float32, "line_follower/desired_velocity_mps"
    )
    measured_velocity_pub = rx.Publisher(
        Float32, "line_follower/measured_velocity_mps"
    )
    distance_error_pub = rx.Publisher(Float32, "line_follower/distance_error_m")
    velocity_phase_pub = rx.Publisher(String, "line_follower/velocity_phase")

    @rx.Subscriber(Float32, aruco_distance_topic)
    def _aruco_distance_callback(self, msg: Float32):
        distance = float(msg.data)
        if np.isfinite(distance) and distance > 0.0:
            self.aruco_distance_samples.append(distance)
            self.aruco_distance = float(np.median(self.aruco_distance_samples))
            self.aruco_distance_stamp_s = self._now_s()

    @rx.Subscriber(String, 'mission/active_controller', qos_pubber)
    def _mission_active(self, msg: String):
        was_active = self.active_controller == str(self.controller_name)
        self.active_controller = msg.data
        is_active = self.active_controller == str(self.controller_name)
        if was_active != is_active:
            self._reset_velocity_controller()

    def on_startup(self):
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_mask = None
        self.latest_centroid = None
        self.line_detected = False
        self.aruco_distance = -1.0
        self.aruco_distance_stamp_s = -1.0
        self.aruco_distance_samples = deque(
            maxlen=max(int(self.aruco_filter_window), 1)
        )
        self.debug_publish_counter = 0
        
        self.steering_error_prev = 0.0
        self.steering_error_integral = 0.0

        self._reset_velocity_controller()

        self.create_subscription(
            Image,
            str(self.image_topic),
            self._image_callback,
            1,
        )

        self.dt_s = max(float(self.dt), 1e-3)
        self.create_timer(self.dt_s, self.loop)
        self.get_logger().info(
            f"Line follower started on image_topic={self.image_topic}, dt={self.dt_s:.3f}s"
        )

    def on_shutdown(self):
        pass

    def _image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")
            return

        centroid, mask = self._extract_line_centroid(frame)
        self.latest_frame = frame
        self.latest_mask = mask
        self.latest_centroid = centroid
        self.line_detected = centroid is not None

    def _extract_line_centroid(self, frame):
        height, _ = frame.shape[:2]
        crop_start = int(np.clip(float(self.crop_start_ratio), 0.0, 0.95) * height)
        roi = frame[crop_start:, :]

        hsv_image = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array(
            [int(self.lower_h), int(self.lower_s), int(self.lower_v)],
            dtype=np.uint8,
        )
        upper = np.array(
            [int(self.upper_h), int(self.upper_s), int(self.upper_v)],
            dtype=np.uint8,
        )
        mask = cv2.inRange(hsv_image, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        min_area = float(self.min_contour_area)
        line = None

        for contour in contours:
            moments = cv2.moments(contour)
            if moments["m00"] > min_area:
                line = (
                    int(moments["m10"] / moments["m00"]),
                    int(moments["m01"] / moments["m00"]) + crop_start,
                )

        return line, mask

    def _calculate_steering(self, normalized_error, dt):
        error_i = (normalized_error + self.steering_error_prev) / 2.0 * dt
        error_d = (normalized_error - self.steering_error_prev) / max(dt, 1e-6)
        self.steering_error_integral += error_i
        self.steering_error_integral = float(
            np.clip(
                self.steering_error_integral,
                -float(self.steering_limit_rad) * 1.5,
                float(self.steering_limit_rad) * 1.5,
            )
        )
        self.steering_error_prev = float(normalized_error)

        steering = -(
            float(self.steering_kp) * normalized_error
            + float(self.steering_ki) * self.steering_error_integral
            + float(self.steering_kd) * error_d
        )
        steering = float(
            np.clip(
                steering,
                -float(self.steering_limit_rad),
                float(self.steering_limit_rad),
            )
        )
        return steering #+ np.deg2rad(16.0)

    def _calculate_velocity(self, normalized_error, dt):
        # Base velocity from line following
        if bool(self.velocity_scale_from_error):
            speed_scale = max(0.25, 1.0 - min(abs(normalized_error), 1.0))
        else:
            speed_scale = 1.0
        base_velocity = min(
            float(self.max_velocity),
            float(self.target_velocity) * speed_scale,
        )

        if not bool(self.use_aruco_stop):
            self._reset_velocity_controller()
            self._publish_velocity_debug(base_velocity, 0.0, 0.0, "tracking")
            return base_velocity

        aruco_is_fresh = (
            self.aruco_distance > 0.0
            and self.aruco_distance_stamp_s > 0.0
            and self._now_s() - self.aruco_distance_stamp_s
            <= max(float(self.aruco_timeout_s), 0.0)
        )
        if not aruco_is_fresh and self.aruco_distance <= 0.0:
            self._reset_velocity_controller()
            self._publish_velocity_debug(base_velocity, 0.0, 0.0, "tracking")
            return base_velocity
        if not aruco_is_fresh:
            self._reset_velocity_controller()
            self._publish_velocity_debug(0.0, 0.0, 0.0, "aruco_stale")
            return 0.0

        ref_dist = float(self.aruco_stop_distance_m)
        dist = float(self.aruco_distance)
        dist_error = dist - ref_dist
        _, _, _, vel = self.localizer.get_state()
        backup_velocity_limit = min(
            float(self.max_velocity),
            float(self.aruco_max_backup_velocity),
        )

        tolerance = max(float(self.dock_tolerance_m), 0.0)
        if tolerance > 0.0 and abs(dist_error) <= tolerance:
            self._reset_velocity_controller()
            self._publish_velocity_debug(0.0, vel, dist_error, "docked")
            return 0.0

        if dist >= float(self.platform_transition_distance_m):
            desired_velocity = max(
                base_velocity,
                min(float(self.ramp_min_velocity), float(self.max_velocity)),
            )
            desired_velocity = min(desired_velocity, float(self.max_velocity))
            forward_command_limit = float(self.max_velocity)
            phase = "ramp"
        else:
            deceleration = max(float(self.approach_deceleration_mps2), 1e-3)
            profile_speed = np.sqrt(
                2.0 * deceleration * max(abs(dist_error) - tolerance, 0.0)
            )
            search_half_width = max(float(self.dock_search_half_width_m), 0.0)
            if self.dock_search_direction == 0:
                self.dock_search_direction = 1 if dist_error >= 0.0 else -1
            elif (
                self.dock_search_direction > 0
                and dist_error <= -search_half_width
            ):
                self._change_dock_search_direction(-1)
            elif (
                self.dock_search_direction < 0
                and dist_error >= search_half_width
            ):
                self._change_dock_search_direction(1)

            if self._now_s() < self.dock_settle_until_s:
                self.previous_velocity_command = 0.0
                self._publish_velocity_debug(0.0, vel, dist_error, "dock_settle")
                return 0.0

            if self.dock_search_direction > 0:
                desired_velocity = min(base_velocity, profile_speed)
                phase = "platform_approach"
            else:
                desired_velocity = -min(backup_velocity_limit, profile_speed)
                phase = "overshoot_recovery"
            forward_command_limit = base_velocity

        vel_error = desired_velocity - vel

        # PID control for velocity
        error_vel_i = (vel_error + self.aruco_velocity_error_prev) / 2.0 * dt
        error_vel_d = (vel_error - self.aruco_velocity_error_prev) / max(dt, 1e-6)
        self.aruco_velocity_integral += error_vel_i
        self.aruco_velocity_integral = float(
            np.clip(
                self.aruco_velocity_integral,
                -float(self.aruco_velocity_integral_limit),
                float(self.aruco_velocity_integral_limit),
            )
        )

        velocity_command = (
            desired_velocity
            + float(self.aruco_velocity_kp) * vel_error
            + float(self.aruco_velocity_ki) * self.aruco_velocity_integral
            + float(self.aruco_velocity_kd) * error_vel_d
        )

        self.aruco_velocity_error_prev = float(vel_error)
        velocity_command = float(
            np.clip(
                velocity_command,
                -backup_velocity_limit,
                forward_command_limit,
            )
        )

        if phase == "platform_approach":
            velocity_command = max(
                velocity_command,
                min(float(self.aruco_min_forward_command), forward_command_limit),
            )
        elif desired_velocity < 0.0:
            velocity_command = min(
                velocity_command,
                -min(float(self.aruco_min_backup_command), backup_velocity_limit),
            )
            self.last_desired_direction = -1
        else:
            self.last_desired_direction = 1

        max_command_step = max(float(self.velocity_command_slew_mps2), 0.0) * dt
        velocity_command = float(
            np.clip(
                velocity_command,
                self.previous_velocity_command - max_command_step,
                self.previous_velocity_command + max_command_step,
            )
        )
        self.previous_velocity_command = velocity_command

        self._publish_velocity_debug(desired_velocity, vel, dist_error, phase)
        return velocity_command

    def _reset_velocity_controller(self):
        self.aruco_velocity_integral = 0.0
        self.aruco_velocity_error_prev = 0.0
        self.last_desired_direction = 0
        self.dock_search_direction = 0
        self.dock_settle_until_s = 0.0
        self.previous_velocity_command = 0.0

    def _change_dock_search_direction(self, direction):
        self.dock_search_direction = 1 if direction > 0 else -1
        self.aruco_velocity_integral = 0.0
        self.aruco_velocity_error_prev = 0.0
        settle_time = max(float(self.dock_settle_time_s), 0.0)
        if self.dock_search_direction < 0:
            settle_time = max(settle_time, float(self.reverse_neutral_time_s))
        self.dock_settle_until_s = self._now_s() + settle_time
        self.previous_velocity_command = 0.0

    def _publish_velocity_debug(
        self, desired_velocity, measured_velocity, distance_error, phase
    ):
        self.desired_velocity_pub.publish(Float32(data=float(desired_velocity)))
        self.measured_velocity_pub.publish(Float32(data=float(measured_velocity)))
        self.distance_error_pub.publish(Float32(data=float(distance_error)))
        self.velocity_phase_pub.publish(String(data=str(phase)))

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def loop(self):
        if self.active_controller != str(self.controller_name):
            return

        frame = self.latest_frame
        if frame is None:
            return

        _, width = frame.shape[:2]
        # Trim for camera mounting offset/yaw: if the car consistently
        # tracks off to one side of the physical line, the image's
        # geometric center isn't the car's true centerline. Tune live via
        # `ros2 param set /self/line_follower steering_bias_px <value>` —
        # sign depends on mount direction, so nudge it one way, check which
        # side the offset shrinks on, then dial in.
        image_center_x = width / 2.0 + float(self.steering_bias_px)

        if self.latest_centroid is None:
            self._publish_status("line_lost")
            self.steering_error_prev = 0.0
            self.steering_error_integral = 0.0
            if bool(self.stop_on_lost_line):
                self.steering_cmd_pub.publish(
                    Float32(data=float(self.lost_line_steering_rad))
                )
                self.velocity_cmd_pub.publish(Float32(data=0.0))
            self._publish_debug_image(frame, None, None)
            return

        cx, cy = self.latest_centroid
        error_px = cx - image_center_x
        normalized_error = error_px / max(image_center_x, 1.0)

        dt = self.dt_s
        steering = self._calculate_steering(normalized_error, dt)
        velocity = self._calculate_velocity(normalized_error, dt)

        self.steering_cmd_pub.publish(Float32(data=float(steering)))
        self.velocity_cmd_pub.publish(Float32(data=float(velocity)))
        self.line_error_pub.publish(Float32(data=float(error_px)))
        self._publish_status(self._get_status_text(velocity))

        centroid_msg = Point()
        centroid_msg.x = float(cx)
        centroid_msg.y = float(cy)
        centroid_msg.z = 0.0
        self.centroid_pub.publish(centroid_msg)

        self._publish_debug_image(frame, (cx, cy), error_px)


    def _publish_status(self, text: str):
        self.status_pub.publish(String(data=text))

    def _get_status_text(self, velocity: float) -> str:
        if not bool(self.use_aruco_stop) or self.aruco_distance <= 0.0 or self.aruco_distance > self.aruco_stop_distance_m + 0.5:
            return "tracking"

        if velocity < 0.0:
            return "backing_up_to_aruco"

        if abs(velocity) < 1e-3:
            return "stopped_at_aruco"

        return "approaching_aruco"

    def _publish_debug_image(self, frame, centroid, error_px):
        if not bool(self.publish_debug_image):
            return

        self.debug_publish_counter += 1
        if self.debug_publish_counter % max(int(self.debug_publish_every_n), 1) != 0:
            return

        debug = frame.copy()
        height, width = debug.shape[:2]
        center_x = width // 2

        cv2.line(debug, (center_x, 0), (center_x, height), (0, 255, 255), 2)

        crop_start = int(np.clip(float(self.crop_start_ratio), 0.0, 0.95) * height)
        cv2.line(debug, (0, crop_start), (width, crop_start), (255, 255, 0), 2)

        if centroid is not None:
            cv2.circle(debug, centroid, 8, (0, 0, 255), -1)
            cv2.putText(
                debug,
                f"error_px={error_px:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            if bool(self.use_aruco_stop) and self.aruco_distance > 0.0:
                cv2.putText(
                    debug,
                    f"aruco_d={self.aruco_distance:.2f} m",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 220, 0),
                    2,
                    cv2.LINE_AA,
                )
        else:
            cv2.putText(
                debug,
                "line lost",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_image_pub.publish(msg)


if __name__ == "__main__":
    line_follower.main()
