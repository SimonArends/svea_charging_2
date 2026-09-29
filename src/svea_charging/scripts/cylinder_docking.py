#!/usr/bin/env python3

from collections import deque
import numpy as np
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

from svea_core import rosonic as rx
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
#from svea_core.interfaces import LocalizationInterface
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    qos_profile_sensor_data,
)

# QoS Profile
qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


# class CylinderDockingLocalizationInterface(LocalizationInterface):
#     def _resolve_base_frame(self, odom=None):
#         base_frame = str(self.localization.base_frame)
#         if base_frame and base_frame != "self/base_link":
#             return base_frame

#         namespace = self.node.get_namespace().strip("/")
#         if namespace:
#             return f"{namespace}/base_link"

#         if odom is not None and odom.child_frame_id:
#             return odom.child_frame_id

#         return base_frame

#     def transform_odom(
#         self,
#         odom,
#         pose_target=None,
#         twist_target=None,
#         timeout_s=0.2,
#     ):
#         resolved_twist_target = (
#             twist_target
#             if twist_target is not None
#             else self._resolve_base_frame(odom)
#         )
#         return super().transform_odom(
#             odom,
#             pose_target=pose_target,
#             twist_target=resolved_twist_target,
#             timeout_s=timeout_s,
#         )


class cylinder_docking(rx.Node):
    # =========================================================================
    # SYSTEM & GENERAL SETTINGS
    # =========================================================================
    is_sim = rx.Parameter(False)
    dt = rx.Parameter(0.05)
    scan_topic = rx.Parameter("/scan")
    stop_on_lost_cylinders = rx.Parameter(True)
    controller_name = rx.Parameter("cylinder_docking")
    active_controller = rx.Parameter("idle")
    left_cylinder_x = rx.Parameter(5.0)
    left_cylinder_y = rx.Parameter(-1.5)
    right_cylinder_x = rx.Parameter(4.7)
    right_cylinder_y = rx.Parameter(-2.1)

    # =========================================================================
    # PERCEPTION & LANDMARK EXTRACTION
    # =========================================================================
    max_detection_distance_m = rx.Parameter(1.5)
    min_cluster_points = rx.Parameter(2)

    # =========================================================================
    # LATERAL (STEERING) CONTROLLER
    # =========================================================================
    steering_kp = rx.Parameter(0.8)
    steering_ki = rx.Parameter(0.3)
    steering_kd = rx.Parameter(0.0)
    steering_limit_rad = rx.Parameter(0.6)
    lost_cylinders_steering_rad = rx.Parameter(0.0)

    # =========================================================================
    # LONGITUDINAL (SPEED) CONTROLLER PARAMETERS
    # =========================================================================
    # --- Velocity Limits & Cruising ---
    target_velocity = rx.Parameter(0.3)            # [m/s] Nominal cruising speed during platform approach
    max_velocity = rx.Parameter(0.4)               # [m/s] Hard global cap for forward speed
    max_backup_velocity = rx.Parameter(0.5)        # [m/s] Hard speed cap when reversing during overshoot recovery

    # --- Distance & Dock Target Thresholds ---
    dock_target_angle_deg = rx.Parameter(90.0)     # [deg] Target cylinder angle representing the final dock stop
    dock_tolerance_deg = rx.Parameter(2.0)         # [deg] Allowed angle error margin to declare "docked" (0 m/s)
    platform_transition_angle_deg = rx.Parameter(40.0) # [deg] Threshold angle separating "ramp" phase from "approach" phase
    dock_search_half_width_deg = rx.Parameter(1.0) # [deg] Hysteresis deadband to prevent chatter when switching forward/reverse
    linear_threshold_deg = rx.Parameter(45.0)       # [deg] Threshold angle below which velocity scales linearly (above this, sqrt profile)

    # --- Speed Profile & Motion Dynamics ---
    ramp_min_velocity = rx.Parameter(0.3)         # [m/s] Minimum velocity enforced on ramp to prevent stalling
    approach_deceleration_mps2 = rx.Parameter(0.05) # [m/s²] Deceleration rate used to compute smooth stopping curve (v = √(2ad))
    velocity_command_slew_mps2 = rx.Parameter(1.0) # [m/s²] Maximum allowed acceleration/jerk step per frame (slew filter)

    # --- Motor Deadband / Anti-Stall Clamps ---
    min_forward_command = rx.Parameter(0.35)       # [m/s] Minimum command output to overcome forward motor static friction
    min_backup_command = rx.Parameter(0.5)         # [m/s] Minimum command output to overcome reverse motor static friction

    # --- Direction Switching & Settling Delays ---
    reverse_neutral_time_s = rx.Parameter(0.25)    # [s] Extra neutral pause added specifically when changing into reverse
    dock_settle_time_s = rx.Parameter(0.30)        # [s] Zero-velocity pause duration during direction changes (prevents gear shock)

    # --- Velocity PID Gains & Anti-Windup ---
    velocity_kp = rx.Parameter(0.5)               # Proportional gain for speed tracking error (desired_vel - measured_vel)
    velocity_ki = rx.Parameter(0.01)               # Integral gain for steady-state speed tracking
    velocity_kd = rx.Parameter(0.0)                # Derivative gain to damp rapid speed oscillations
    velocity_integral_limit = rx.Parameter(0.4)    # Anti-windup cap applied to velocity error integrator

    # =========================================================================
    # INTERFACES, PUBLISHERS & SUBSCRIBERS
    # =========================================================================
    #localizer = CylinderDockingLocalizationInterface()

    steering_cmd_topic = rx.Parameter("cylinder_docking/cmd_steering_rad")
    velocity_cmd_topic = rx.Parameter("cylinder_docking/cmd_velocity_mps")

    steering_cmd_pub = rx.Publisher(Float32, steering_cmd_topic)
    velocity_cmd_pub = rx.Publisher(Float32, velocity_cmd_topic)
    angular_error_pub = rx.Publisher(Float32, "cylinder_docking/angular_error_deg")
    opening_angle_pub = rx.Publisher(Float32, "cylinder_docking/opening_angle_deg")
    status_pub = rx.Publisher(String, "cylinder_docking/status")
    cylinder_distance_pub = rx.Publisher(Float32, "cylinder_docking/cylinder_distance_m")
    desired_velocity_pub = rx.Publisher(Float32, "cylinder_docking/desired_velocity_mps")
    measured_velocity_pub = rx.Publisher(Float32, "cylinder_docking/measured_velocity_mps")
    angle_error_deg_pub = rx.Publisher(Float32, "cylinder_docking/angle_error_deg")
    velocity_phase_pub = rx.Publisher(String, "cylinder_docking/velocity_phase")

    left_cylinder_pub = rx.Publisher(Point, "cylinder_docking/left_cylinder")
    right_cylinder_pub = rx.Publisher(Point, "cylinder_docking/right_cylinder")


    @rx.Subscriber(String, 'mission/active_controller', qos_pubber)
    def _mission_active(self, msg: String):
        was_active = self.active_controller == str(self.controller_name)
        self.active_controller = msg.data
        is_active = self.active_controller == str(self.controller_name)
        if was_active != is_active:
            self._reset_velocity_controller()

    def on_startup(self):
        if self.is_sim:
            self.sim_odom_sub = self.create_subscription(
                Odometry,
                "odometry/local",
                self.odom_sim_cb,
                qos_pubber,
            )
        else:
            self.odom_sub = self.create_subscription(
                Odometry,
                "/svea3/odom",
                self.odom_cb,
                qos_profile_sensor_data,
            )
        self.x = None
        self.y = None
        self.vel = None
        self.left_cylinder_pos = None
        self.right_cylinder_pos = None
        self.cylinders_detected = False

        self.steering_error_prev = 0.0
        self.steering_error_integral = 0.0

        self._reset_velocity_controller()

        self.create_subscription(
            LaserScan,
            str(self.scan_topic),
            self._scan_callback,
            1,
        )

        self.dt_s = max(float(self.dt), 1e-3)
        self.create_timer(self.dt_s, self.loop)
        self.get_logger().info(
            f"Cylinder Docking started on scan_topic={self.scan_topic}, dt={self.dt_s:.3f}s"
        )

    def on_shutdown(self):
        pass

    def odom_sim_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        odom_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        odom_yaw = math.atan2(math.sin(odom_yaw), math.cos(odom_yaw))  # wrap to [-pi, pi]
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.yaw = float(odom_yaw)
        self.vel = float(msg.twist.twist.linear.x)


    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        odom_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2] - math.pi/2
        odom_yaw = math.atan2(math.sin(odom_yaw), math.cos(odom_yaw))  # wrap to [-pi, pi]
        self.x = float(msg.pose.pose.position.y)
        self.y = -float(msg.pose.pose.position.x)
        self.yaw = float(odom_yaw)
        self.vel = float(msg.twist.twist.linear.x)

    def _cylinder_center_in_map(self, left, right):
        """
        Transform the midpoint between the detected cylinders from
        the robot frame into the map frame.
        Args:
            left:  Cylinder dictionary containing "x" and "y".
            right: Cylinder dictionary containing "x" and "y".
        Returns:
            (map_x, map_y) or None if robot pose is unavailable.
        """
        # We need a valid robot pose
        if self.x is None or self.y is None or self.yaw is None:
            return None
        # Midpoint between the cylinders in robot frame
        center_x_robot = (left["x"] + right["x"]) / 2.0
        center_y_robot = (left["y"] + right["y"]) / 2.0
        # Robot -> map transformation
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        center_x_map = (
            self.x
            + cos_yaw * center_x_robot
            - sin_yaw * center_y_robot
        )
        center_y_map = (
            self.y
            + sin_yaw * center_x_robot
            + cos_yaw * center_y_robot
        )
        return center_x_map, center_y_map


    def _scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        num_points = len(ranges)
        angles = msg.angle_min + np.arange(num_points) * msg.angle_increment

        # ============================================================
        # Parameters
        # ============================================================
        max_dist = float(self.max_detection_distance_m)
        # Minimum valid LiDAR range
        min_dist = max(float(msg.range_min), 0.05)

        # Cylinders are 8 cm diameter.
        # Allow some tolerance because the LiDAR will not necessarily
        # observe the entire cylinder.
        max_object_width = 0.25       # 15 cm
        # Minimum number of LiDAR points required for an object
        min_cluster_points = 1
        # Consecutive points farther apart than this are considered
        # different objects.
        max_point_gap = 0.25          # 12 cm
        # Ignore objects too close to the robot
        min_cylinder_x = -0.30         # 30 cm
        # Cylinder must be sufficiently to the left/right of robot
        min_side_distance = 0.05     # 25 cm
        # The two cylinders should be roughly aligned in front of robot
        max_pair_x_difference = 0.4   # 75 cm
        # Expected separation between the two cylinders
        min_cylinder_separation = 0.4
        max_cylinder_separation = 0.9
        # =========================================================================
        # MAP-BASED CYLINDER LOCATION VALIDATION
        # =========================================================================
        target_center_x = 0.0
        target_center_y = 0.0
        target_center_tolerance_m = 0.5

        # ============================================================
        # Basic valid LiDAR mask
        # ============================================================

        valid_mask = (
            np.isfinite(ranges) &
            (ranges >= min_dist) &
            (ranges <= max_dist) &
            (np.abs(angles) <= np.deg2rad(110.0))
        )

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # ============================================================
        # Find contiguous physical objects
        # ============================================================
        clusters = []
        current_cluster = []
        for i in range(num_points):
            # Invalid measurement -> end current object
            if not valid_mask[i]:
                if current_cluster:
                    clusters.append(current_cluster)
                    current_cluster = []
                continue

            # First point of a new object
            if not current_cluster:
                current_cluster.append(i)
                continue

            # Distance between this point and previous point
            prev_i = current_cluster[-1]
            point_gap = np.hypot(
                x[i] - x[prev_i],
                y[i] - y[prev_i]
            )
            # Large physical gap -> new object
            if point_gap > max_point_gap:
                clusters.append(current_cluster)
                current_cluster = [i]
            else:
                current_cluster.append(i)
        # Don't forget final cluster
        if current_cluster:
            clusters.append(current_cluster)

        # ============================================================
        # Find small objects that could be cylinders
        # ============================================================
        cylinder_candidates = []
        for cluster in clusters:
            # Too few points to be useful
            if len(cluster) < min_cluster_points:
                continue
            cluster_x = x[cluster]
            cluster_y = y[cluster]
            # --------------------------------------------------------
            # Physical size of the cluster
            #
            # Use maximum pairwise distance rather than only
            # max(x)-min(x), because the object may be angled.
            # --------------------------------------------------------
            dx = cluster_x[:, None] - cluster_x[None, :]
            dy = cluster_y[:, None] - cluster_y[None, :]
            distances = np.hypot(dx, dy)
            object_width = np.max(distances)
            # If the object is substantially larger than our
            # 8 cm cylinder, reject it.
            if object_width > max_object_width:
                continue
            # --------------------------------------------------------
            # Object center
            # --------------------------------------------------------
            center_x = float(np.mean(cluster_x))
            center_y = float(np.mean(cluster_y))
            # Ignore objects behind/too close to robot
            if center_x < min_cylinder_x:
                continue
            # Ignore objects close to the center line.
            # We want one cylinder on each side.
            if abs(center_y) < min_side_distance:
                continue
            cylinder_candidates.append({
                "indices": cluster,
                "x": center_x,
                "y": center_y,
                "width": float(object_width),
                "num_points": len(cluster),
            })

        # ============================================================
        # Split candidates into left/right
        # ============================================================
        left_candidates = [
            c for c in cylinder_candidates
            if c["y"] > min_side_distance
        ]
        right_candidates = [
            c for c in cylinder_candidates
            if c["y"] < -min_side_distance
        ]
        # ============================================================
        # Find a valid pair
        # ============================================================
        cylinders_detected = False
        best_pair = None
        for left in left_candidates:
            for right in right_candidates:
                # ----------------------------------------------------
                # Cylinders should be roughly at the same forward
                # distance from the robot.
                # ----------------------------------------------------
                x_difference = abs(left["x"] - right["x"])
                if x_difference > max_pair_x_difference:
                    continue
                # ----------------------------------------------------
                # Distance between the cylinders
                # ----------------------------------------------------
                cylinder_separation = abs(left["y"] - right["y"])
                if cylinder_separation < min_cylinder_separation:
                    continue
                if cylinder_separation > max_cylinder_separation:
                    continue
                # ----------------------------------------------------
                # Check cylinder midpoint against target map position
                # ----------------------------------------------------
                center_map = self._cylinder_center_in_map(left, right)
                # No valid robot pose -> cannot perform map validation
                if center_map is None:
                    continue
                center_x_map, center_y_map = center_map
                center_error = math.hypot(
                    center_x_map - target_center_x,
                    center_y_map - target_center_y,
                )
                # Reject this cylinder pair if its center is outside
                # the allowed map-coordinate tolerance.
                if center_error > target_center_tolerance_m:
                    continue
                # ----------------------------------------------------
                # Valid pair found
                # ----------------------------------------------------
                cylinders_detected = True
                best_pair = (left, right)
                break
            if cylinders_detected:
                break
        # ============================================================
        # Output
        # ============================================================
        self.cylinders_detected = cylinders_detected
        if best_pair is not None:

            left, right = best_pair

            cx_left = left["x"]
            cy_left = left["y"]

            cx_right = right["x"]
            cy_right = right["y"]

            self.left_cylinder_pos = (cx_left, cy_left)
            self.right_cylinder_pos = (cx_right, cy_right)

            self.left_cylinder_pub.publish(Point(x=cx_left, y=cy_left, z=0.0))
            self.right_cylinder_pub.publish(Point(x=cx_right, y=cy_right, z=0.0))

    def _calculate_steering(self, angular_error, theta_L, theta_R, dt):
        # Safety Check: If cylinders are behind the front axle/base frame, zero steering
        if theta_R < np.deg2rad(-90.0) or theta_L > np.deg2rad(90.0):
            self.steering_error_integral = 0.0
            self.steering_error_prev = 0.0
            return 0.0

        # Normal Forward PID Logic
        error_i = (angular_error + self.steering_error_prev) / 2.0 * dt
        error_d = (angular_error - self.steering_error_prev) / max(dt, 1e-6)
        
        self.steering_error_integral += error_i
        limit_rad = float(self.steering_limit_rad)
        self.steering_error_integral = float(
            np.clip(self.steering_error_integral, -limit_rad * 2, limit_rad * 2)
        )
        self.steering_error_prev = float(angular_error)

        steering = (
            float(self.steering_kp) * angular_error
            + float(self.steering_ki) * self.steering_error_integral
            + float(self.steering_kd) * error_d
        )

        return float(np.clip(steering, -limit_rad, limit_rad))

    def _calculate_velocity(self, opening_angle_deg, dt):
        base_velocity = float(self.target_velocity)

        ref_angle = float(self.dock_target_angle_deg)
        angle_error = ref_angle - opening_angle_deg  
        
        vel = self.vel
        
        # backup_velocity_limit = min(
        #     float(self.max_velocity),
        #     float(self.max_backup_velocity),
        # )
        backup_velocity_limit = self.max_backup_velocity

        tolerance = max(float(self.dock_tolerance_deg), 0.0)
        if tolerance > 0.0 and abs(angle_error) <= tolerance:
            self._reset_velocity_controller()
            self._publish_velocity_debug(0.0, vel, angle_error, "docked")
            return 0.0

        if opening_angle_deg <= float(self.platform_transition_angle_deg):
            desired_velocity = max(
                base_velocity,
                min(float(self.ramp_min_velocity), float(self.max_velocity)),
            )
            desired_velocity = min(desired_velocity, float(self.max_velocity))
            forward_command_limit = float(self.max_velocity)
            phase = "ramp"
        
        else:
            search_half_width = max(float(self.dock_search_half_width_deg), 0.0)
            if self.dock_search_direction == 0:
                self.dock_search_direction = 1 if angle_error >= 0.0 else -1
            elif (
                self.dock_search_direction > 0
                and angle_error <= -search_half_width
            ):
                self._change_dock_search_direction(-1)
            elif (
                self.dock_search_direction < 0
                and angle_error >= search_half_width
            ):
                self._change_dock_search_direction(1)

            if self._now_s() < self.dock_settle_until_s:
                self.previous_velocity_command = 0.0
                self._publish_velocity_debug(0.0, vel, angle_error, "dock_settle")
                return 0.0

            # Smooth Kinematic Profile (Linear near zero, sqrt further away)
            deceleration = max(float(self.approach_deceleration_mps2), 1e-3)
            error_deg_mag = max(abs(angle_error) - tolerance, 0.0)

            kp_approach = np.sqrt(2.0 * deceleration / max(np.deg2rad(self.linear_threshold_deg), 1e-4))

            if error_deg_mag <= self.linear_threshold_deg:
                profile_speed = kp_approach * np.deg2rad(error_deg_mag)
            else:
                profile_speed = np.sqrt(
                    2.0 * deceleration * np.deg2rad(error_deg_mag)
                )

            if self.dock_search_direction > 0:
                desired_velocity = min(base_velocity, profile_speed)
                phase = "platform_approach"
            else:
                desired_velocity = -min(backup_velocity_limit, profile_speed)
                phase = "overshoot_recovery"
            forward_command_limit = base_velocity

        vel_error = desired_velocity - vel

        error_vel_i = (vel_error + self.velocity_error_prev) / 2.0 * dt
        error_vel_d = (vel_error - self.velocity_error_prev) / max(dt, 1e-6)
        
        self.velocity_integral += error_vel_i
        self.velocity_integral = float(
            np.clip(
                self.velocity_integral,
                -float(self.velocity_integral_limit),
                float(self.velocity_integral_limit),
            )
        )

        velocity_command = (
            desired_velocity
            + float(self.velocity_kp) * vel_error
            + float(self.velocity_ki) * self.velocity_integral
            + float(self.velocity_kd) * error_vel_d
        )

        self.velocity_error_prev = float(vel_error)
        velocity_command = float(
            np.clip(
                velocity_command,
                -backup_velocity_limit,
                forward_command_limit,
            )
        )

        # To avoid stalling 
        if phase == "platform_approach":
            velocity_command = max(
                velocity_command,
                min(float(self.min_forward_command), forward_command_limit),
            )
        elif desired_velocity < 0.0:
            velocity_command = min(
                velocity_command,
                -min(float(self.min_backup_command), backup_velocity_limit),
            )

        max_command_step = max(float(self.velocity_command_slew_mps2), 0.0) * dt
        velocity_command = float(
            np.clip(
                velocity_command,
                self.previous_velocity_command - max_command_step,
                self.previous_velocity_command + max_command_step,
            )
        )
        self.previous_velocity_command = velocity_command

        self._publish_velocity_debug(desired_velocity, vel, angle_error, phase)
        return velocity_command

    def _reset_velocity_controller(self):
        self.velocity_integral = 0.0
        self.velocity_error_prev = 0.0
        self.dock_search_direction = 0
        self.dock_settle_until_s = 0.0
        self.previous_velocity_command = 0.0

    def _change_dock_search_direction(self, direction):
        self.dock_search_direction = 1 if direction > 0 else -1
        self.velocity_integral = 0.0
        self.velocity_error_prev = 0.0
        settle_time = max(float(self.dock_settle_time_s), 0.0)
        if self.dock_search_direction < 0:
            settle_time = max(settle_time, float(self.reverse_neutral_time_s))
        self.dock_settle_until_s = self._now_s() + settle_time
        self.previous_velocity_command = 0.0

    def _publish_velocity_debug(
        self, desired_velocity, measured_velocity, angle_error, phase
    ):
        self.desired_velocity_pub.publish(Float32(data=float(desired_velocity)))
        self.measured_velocity_pub.publish(Float32(data=float(measured_velocity)))
        self.angle_error_deg_pub.publish(Float32(data=float(angle_error)))
        self.velocity_phase_pub.publish(String(data=str(phase)))

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9


    def loop(self):
        if self.active_controller != str(self.controller_name):
            if not self.cylinders_detected:
                self.cylinder_distance_pub.publish(Float32(data=float(-1.0)))
            else:
                x_L, y_L = self.left_cylinder_pos
                x_R, y_R = self.right_cylinder_pos
                cylinder_distance = (x_L + x_R) / 2.0
                self.cylinder_distance_pub.publish(Float32(data=float(cylinder_distance)))
            return

        if not self.cylinders_detected:
            self.get_logger().warn("Cylinders lost, stopping the robot.")
            self.status_pub.publish(String(data="cylinders_lost"))
            self.steering_error_prev = 0.0
            self.steering_error_integral = 0.0
            # self.actuation.send_control(0.5, 0.2)
            self.cylinder_distance_pub.publish(Float32(data=float(-1.0)))
            if bool(self.stop_on_lost_cylinders):
                self.steering_cmd_pub.publish(
                    Float32(data=float(self.lost_cylinders_steering_rad))
                )
                self.velocity_cmd_pub.publish(Float32(data=0.0))
            return

        dt = self.dt_s
        
        x_L, y_L = self.left_cylinder_pos
        x_R, y_R = self.right_cylinder_pos
        # self.left_cylinder_pub.publish(Point(x=x_L, y=y_L, z=0.0))
        # self.right_cylinder_pub.publish(Point(x=x_R, y=y_R, z=0.0))
        cylinder_distance = (x_L + x_R) / 2.0

        theta_L = np.arctan2(y_L, x_L)
        theta_R = np.arctan2(y_R, x_R)
        # self.get_logger().info(
        #     f"Theta_L: {np.degrees(theta_L):.2f} deg, "
        #     f"Theta_R: {np.degrees(theta_R):.2f} deg"
        # )

        angular_error = theta_L + theta_R
        # self.get_logger().info(
        #     f"Angular Error: {np.degrees(angular_error):.2f} deg"
        # )
        opening_angle_rad = (theta_L - theta_R) / 2.0
        opening_angle_deg = np.degrees(opening_angle_rad)

        steering = self._calculate_steering(angular_error, theta_L, theta_R, dt) + np.deg2rad(0.5)
        velocity = self._calculate_velocity(opening_angle_deg, dt)

        # self.get_logger().info(
        #     f"2. Steering: {steering:.3f} rad, Velocity: {velocity:.3f} m/s"
        # )

        self.steering_cmd_pub.publish(Float32(data=float(steering)))
        self.velocity_cmd_pub.publish(Float32(data=float(velocity)))
        
        self.angular_error_pub.publish(Float32(data=float(np.degrees(angular_error))))
        self.opening_angle_pub.publish(Float32(data=float(opening_angle_deg)))
        self.status_pub.publish(String(data=self._get_status_text(velocity)))
        self.cylinder_distance_pub.publish(Float32(data=float(cylinder_distance)))

    def _get_status_text(self, velocity: float) -> str:
        if velocity < 0.0:
            return "backing_up_to_dock"
        if abs(velocity) < 1e-3:
            return "stopped_at_dock"
        return "approaching_dock"


if __name__ == "__main__":
    cylinder_docking.main()