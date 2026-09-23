#!/usr/bin/env python3

from dataclasses import dataclass
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from svea_core import rosonic as rx
from svea_core.interfaces import ActuationInterface
#from svea_core.interfaces import LocalizationInterface

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


@dataclass
class ControllerCommand:
    steering: float = 0.0
    velocity: float = 0.0
    stamp_s: float = 0.0


class control_mux(rx.Node):
    is_sim = rx.Parameter(True)
    name_svea = rx.Parameter("svea_a")
    other_svea = rx.Parameter("svea_b")
    controller_timeout_s = rx.Parameter(0.3)
    output_hz = rx.Parameter(20.0)
    active_controller = rx.Parameter("idle")
    charging_arm_topic = rx.Parameter("charging_arm")
    charging_arm_active_xtr1 = rx.Parameter(100.0)
    charging_arm_inactive_xtr1 = rx.Parameter(0.0)
    #localizer = LocalizationInterface()
    actuation = ActuationInterface()

    def _other_region_cb(self, msg: String):
        data = msg.data.split("|")

        self.other_region = data[0]
        self.other_region_entry_time = float(data[1])

        if self.regions_overlap(self.region, self.other_region):

            # Other robot entered first -> I must stop
            if self.other_region_entry_time < self.region_entry_time:
                self.waiting = True

            # I entered first -> I keep going
            else:
                self.waiting = False

    # @rx.Subscriber(Odometry, "/svea3/odom")
    # def _odometry_cb(self, msg: Odometry):
    #     x = float(msg.pose.pose.position.y)
    #     y = -float(msg.pose.pose.position.x)

    #     new_region = self.get_regions(x, y)

    #     if new_region != self.region:
    #         self.region = new_region
    #         self.region_entry_time = self._now_s()

    @rx.Subscriber(String, "mission/active_controller", qos_pubber)
    def _active_controller_cb(self, msg: String):
        self.active_controller = msg.data

    @rx.Subscriber(Bool, charging_arm_topic, qos_pubber)
    def _charging_arm_cb(self, msg: Bool):
        self.charging_arm_enabled = bool(msg.data)

    @rx.Subscriber(Float32, "stanley/cmd_steering_rad", qos_pubber)
    def _stanley_steering_cb(self, msg: Float32):
        self.stanley_cmd.steering = float(msg.data)
        self.stanley_cmd.stamp_s = self._now_s()

    @rx.Subscriber(Float32, "stanley/cmd_velocity_mps", qos_pubber)
    def _stanley_velocity_cb(self, msg: Float32):
        self.stanley_cmd.velocity = float(msg.data)
        self.stanley_cmd.stamp_s = self._now_s()

    @rx.Subscriber(Float32, "cylinder_docking/cmd_steering_rad", qos_pubber)
    def _cylinder_steering_cb(self, msg: Float32):
        self.cylinder_cmd.steering = float(msg.data)
        self.cylinder_cmd.stamp_s = self._now_s()

    @rx.Subscriber(Float32, "cylinder_docking/cmd_velocity_mps", qos_pubber)
    def _cylinder_velocity_cb(self, msg: Float32):
        self.cylinder_cmd.velocity = float(msg.data)
        self.cylinder_cmd.stamp_s = self._now_s()

    @rx.Subscriber(Float32, "line_follower/cmd_steering_rad", qos_pubber)
    def _line_steering_cb(self, msg: Float32):
        self.line_cmd.steering = float(msg.data)
        self.line_cmd.stamp_s = self._now_s()

    @rx.Subscriber(Float32, "line_follower/cmd_velocity_mps", qos_pubber)
    def _line_velocity_cb(self, msg: Float32):
        self.line_cmd.velocity = float(msg.data)
        self.line_cmd.stamp_s = self._now_s()

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
                "/svea_3/odom",
                self.odom_cb,
                qos_pubber,
            )
        self.region = ""
        self.other_region = ""

        self.region_entry_time = self._now_s()
        self.other_region_entry_time = 0.0

        self.waiting = False
        self.stanley_cmd = ControllerCommand()
        self.cylinder_cmd = ControllerCommand()
        self.line_cmd = ControllerCommand()
        self.charging_arm_enabled = False

        region_topic = f"/{self.name_svea}/region"
        other_region_topic = f"/{self.other_svea}/region"

        self.region_pub = self.create_publisher(
            String,
            region_topic,
            qos_pubber,
        )

        self.other_region_sub = self.create_subscription(
            String,
            other_region_topic,
            self._other_region_cb,
            qos_pubber,
        )

        period = 1.0 / max(float(self.output_hz), 1.0)
        self.create_timer(period, self.loop)

        self.get_logger().info("Control mux started")

    def odom_sim_cb(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)

        new_region = self.get_regions(x, y)

        if new_region != self.region:
            self.region = new_region
            self.region_entry_time = self._now_s()

    def odom_cb(self, msg: Odometry):
        x = float(msg.pose.pose.position.y)
        y = -float(msg.pose.pose.position.x)

        new_region = self.get_regions(x, y)

        if new_region != self.region:
            self.region = new_region
            self.region_entry_time = self._now_s()

    def loop(self):
        # x = self.localizer.get_x()
        # y = self.localizer.get_y()

        # new_region = self.get_regions(x, y)

        # if new_region != self.region:
        #     self.region = new_region
        #     self.region_entry_time = self._now_s()

        data = f"{self.region}|{self.region_entry_time}"
        self.region_pub.publish(String(data=data))

        if self.waiting:
            if not self.regions_overlap(self.region, self.other_region):
                self.waiting = False
            else:
                self.actuation.send_control(0.0, 0.0)
                return
        cmd = self._get_selected_command()
        xtr1 = (
            float(self.charging_arm_active_xtr1)
            if self.charging_arm_enabled
            else float(self.charging_arm_inactive_xtr1)
        )
        self.actuation.send_xtr(xtr1=xtr1)
        if self.is_sim:
            self.actuation.send_control(cmd.steering, cmd.velocity)
        else:
            self.actuation.send_control(cmd.steering, cmd.velocity)

    def _get_selected_command(self) -> ControllerCommand:
        active = str(self.active_controller)
        if active == "transport_stanley":
            return self._validated_command(self.stanley_cmd)
        if active == "stanley":
            return self._validated_command(self.stanley_cmd)
        if active == "cylinder_docking":
            return self._validated_command(self.cylinder_cmd)
        if active == "line_follower":
            return self._validated_command(self.line_cmd)
        if active == "post_stanley":
            return self._validated_command(self.stanley_cmd)
        return ControllerCommand()

    def _validated_command(self, cmd: ControllerCommand) -> ControllerCommand:
        if self._now_s() - cmd.stamp_s > float(self.controller_timeout_s):
            return ControllerCommand()
        return cmd

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def get_regions(self, x, y):
        regions = []
        # 4 L-shaped regions: (x_min, x_max, y_min, y_max, cutout...)
        l_regions = {
            "C1": (-2.5, 0.0, 0.0, 2.5, -1.7, 0.0, 0.0, 1.05),
            "C2": (0.0, 2.5, 0.0, 2.5, 0.0, 1.7, 0.0, 1.05),
            "C3": (0.7, 2.5, -2.5, 0.0, 0.7, 1.7, -0.95, 0.0),
            "C4": (-2.5, 0.7, -2.5, 0.0, -1.7, 0.7, -0.95, 0.0),
        }

        # 3 rectangles: (x_min, x_max, y_min, y_max)
        rectangles = {
            "A": (-1.7, -0.9, -0.95, 1.05),
            "charger": (-0.9, 0.9, -0.95, 1.05),
            "B": (0.9, 1.7, -0.95, 1.05),
        }

        # Check L-shaped regions
        for name, (xmin, xmax, ymin, ymax,
                cxmin, cxmax, cymin, cymax) in l_regions.items():
            in_outer = xmin <= x <= xmax and ymin <= y <= ymax
            in_cutout = cxmin <= x <= cxmax and cymin <= y <= cymax
            if in_outer and not in_cutout:
                regions.append(name)

        # Check rectangles
        for name, (xmin, xmax, ymin, ymax) in rectangles.items():
            if xmin <= x <= xmax and ymin <= y <= ymax:
                regions.append(name)

        return " ".join(regions)

    def regions_overlap(self, region_a, region_b):
        return bool(set(region_a.split()) & set(region_b.split()))

if __name__ == "__main__":
    control_mux.main()