#!/usr/bin/env python3

from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import BatteryState
import time

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
battery_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

from svea_core import rosonic as rx
from svea_charging.behaviourTree.behaviourTree import ChargingMissionTree, MissionBlackboard


qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)



class bt_runner(rx.Node):
    tick_hz = rx.Parameter(20.0)
    switch_distance_m = rx.Parameter(3.8)
    dock_distance_m = rx.Parameter(1.6)
    charge_done_level = rx.Parameter(95.0)
    charge_demo_duration_s = rx.Parameter(10.0)
    charging_arm_topic = rx.Parameter("/charging_arm")

    dist_to_goal_topic = rx.Parameter("dist_to_goal")
    aruco_distance_topic = rx.Parameter("aruco/distance_m")
    line_status_topic = rx.Parameter("line_follower/status")
    battery_charging_topic = rx.Parameter("/lli/battery/state")

    active_controller_pub = rx.Publisher(String, "mission/active_controller", qos_pubber)
    phase_pub = rx.Publisher(String, "mission/phase", qos_pubber)
    tree_status_pub = rx.Publisher(String, "mission/tree_status", qos_pubber)
    charging_arm_pub = rx.Publisher(Bool, charging_arm_topic, qos_pubber)

    timer_function_pub = rx.Publisher(Float32, "mission/timer_function", qos_pubber)

    @rx.Subscriber(Float32, dist_to_goal_topic)
    def _dist_to_goal_cb(self, msg: Float32):
        self.bb.dist_to_station = float(msg.data)

    @rx.Subscriber(Float32, aruco_distance_topic)
    def _aruco_distance_cb(self, msg: Float32):
        distance = float(msg.data)
        if distance > 0.0:
            self.bb.aruco_distance = distance
            self.bb.charger_visible = True
        else:
            self.bb.aruco_distance = None
            self.bb.charger_visible = False

    @rx.Subscriber(String, line_status_topic, qos_pubber)
    def _line_status_cb(self, msg: String):
        status = msg.data
        self.bb.line_visible = status not in {"line_lost", "idle"}

    @rx.Subscriber(BatteryState, battery_charging_topic, battery_qos)
    def _battery_charging_cb(self, msg: BatteryState):
        self.bb.battery_current = float(msg.current)
        self.bb.battery_voltage = float(msg.voltage)
        if not self.bb.charge_demo_complete:
            self.bb.battery_level = float(msg.percentage) * 100
        # self.get_logger().info(f'battery current: {self.bb.battery_current}')

    def on_startup(self):
        self.bb = MissionBlackboard(
            switch_distance_m=float(self.switch_distance_m),
            dock_distance_m=float(self.dock_distance_m),
            charge_done_level=float(self.charge_done_level),
            charge_demo_duration_s=float(self.charge_demo_duration_s),
        )
        self.tree = ChargingMissionTree(self.bb, self._set_charging_arm)
        period = 1.0 / self.tick_hz
        self.create_timer(period, self.loop)
        self.get_logger().info(
            "BT runner started "
            f"(switch={self.bb.switch_distance_m:.2f} m, dock={self.bb.dock_distance_m:.2f} m, "
            f"charge_demo={self.bb.charge_demo_duration_s:.1f} s, "
            f"charge_done={self.bb.charge_done_level:.1f}%)"
        )

    def _set_charging_arm(self, enabled: bool):
        self.charging_arm_pub.publish(Bool(data=enabled))

    def loop(self):
        status = self.tree.tick()
        self.active_controller_pub.publish(String(data=self.bb.active_controller))
        self.phase_pub.publish(String(data=self.bb.mission_phase))
        self.tree_status_pub.publish(String(data=status))
        if self.bb.charging_active:
            elapsed_s = time.monotonic() - self.bb.charge_started_at
            self.timer_function_pub.publish(Float32(data=elapsed_s))



if __name__ == "__main__":
    bt_runner.main()
