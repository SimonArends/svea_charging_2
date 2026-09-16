#!/usr/bin/env python3
import math
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import BatteryState

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
    switch_distance_m = rx.Parameter(2.5)
    docking_exit_distance_m = rx.Parameter(2.75)
    dock_distance_m = rx.Parameter(0.622)
    charge_start_voltage = rx.Parameter(12.2)
    charge_done_voltage = rx.Parameter(12.6)
    charge_voltage_confirm_s = rx.Parameter(3.0)
    charging_arm_topic = rx.Parameter("charging_arm")

    dist_to_goal_topic = rx.Parameter("dist_to_goal")
    aruco_distance_topic = rx.Parameter("aruco/distance_m")
    line_status_topic = rx.Parameter("line_follower/status")
    battery_charging_topic = rx.Parameter("mavros/battery")  
    stanley_status_topic = rx.Parameter("outdoor_stanley/status")
    transport_location_topic = rx.Parameter("outdoor_stanley/location")
    charge_permission_topic = rx.Parameter("mission/charge_permission")

    active_controller_pub = rx.Publisher(String, "mission/active_controller", qos_pubber)
    phase_pub = rx.Publisher(String, "mission/phase", qos_pubber)
    tree_status_pub = rx.Publisher(String, "mission/tree_status", qos_pubber)
    charging_arm_pub = rx.Publisher(Bool, charging_arm_topic, qos_pubber)
    charging_status_pub = rx.Publisher(Bool, "mission/charging_status", qos_pubber)
    need_charging_pub = rx.Publisher(Bool, "mission/need_charging", qos_pubber)

    @rx.Subscriber(Float32, dist_to_goal_topic)
    def _dist_to_goal_cb(self, msg: Float32):
        self.bb.dist_to_station = float(msg.data)
    
    @rx.Subscriber(String, stanley_status_topic)
    def _stanley_status(self, msg: String):
    	self.bb.stanley_status = msg.data
    	
    @rx.Subscriber(String, transport_location_topic)
    def _transport_location(self, msg: String):
    	self.bb.transport_location = msg.data	

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

    @rx.Subscriber(Bool, charge_permission_topic, qos_pubber)
    def _charge_permission_cb(self, msg: Bool):
    	self.bb.charge_permission = msg.data
    
    def on_startup(self):
        self.bb = MissionBlackboard(
            switch_distance_m=float(self.switch_distance_m),
            docking_exit_distance_m=float(self.docking_exit_distance_m),
            dock_distance_m=float(self.dock_distance_m),
            charge_start_voltage=float(self.charge_start_voltage),
            charge_done_voltage=float(self.charge_done_voltage),
            charge_voltage_confirm_s=float(self.charge_voltage_confirm_s),
        )
        self.tree = ChargingMissionTree(self.bb, self._set_charging_arm)
        self._set_charging_arm(False)
        period = 1.0 / self.tick_hz
        self.create_timer(period, self.loop)
        self.get_logger().info(
            "BT runner started "
            f"(switch={self.bb.switch_distance_m:.2f} m, "
            f"exit={self.bb.docking_exit_distance_m:.2f} m, "
            f"dock={self.bb.dock_distance_m:.2f} m, "
            f"charge_start={self.bb.charge_start_voltage:.2f} V, "
            f"charge_done={self.bb.charge_done_voltage:.2f} V, "
            f"confirm={self.bb.charge_voltage_confirm_s:.1f} s)"
        )

    def _set_charging_arm(self, enabled: bool):
        self.charging_arm_pub.publish(Bool(data=enabled))

    def loop(self):
        status = self.tree.tick()
        self.active_controller_pub.publish(String(data=self.bb.active_controller))
        self.phase_pub.publish(String(data=self.bb.mission_phase))
        self.tree_status_pub.publish(String(data=status))
        self.charging_status_pub.publish(Bool(data=self.bb.charging_active))
        self.need_charging_pub.publish(Bool(data=self.bb.need_charging))

if __name__ == "__main__":
    bt_runner.main()
