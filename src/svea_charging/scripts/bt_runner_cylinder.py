#!/usr/bin/env python3

from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry

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
from svea_charging.behaviourTree.cylinderBehaviourTree import ChargingMissionTree, MissionBlackboard


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
    charging_arm_topic = rx.Parameter("/charging_arm")
    x_switching_point = rx.Parameter(23.47)
    y_switching_point = rx.Parameter(-21.63)

    dist_to_goal_topic = rx.Parameter("dist_to_goal")
    aruco_distance_topic = rx.Parameter("aruco/distance_m")
    battery_charging_topic = rx.Parameter("/self/mavros/battery")
    odometry_topic = rx.Parameter("odometry/global")

    # --- Cylinder-specific parameters ---
    cylinder_status_topic = rx.Parameter("cylinder_docking/status")
    cylinder_distance_topic = rx.Parameter("cylinder_docking/cylinder_distance_m")

    active_controller_pub = rx.Publisher(String, "mission/active_controller", qos_pubber)
    phase_pub = rx.Publisher(String, "mission/phase", qos_pubber)
    tree_status_pub = rx.Publisher(String, "mission/tree_status", qos_pubber)
    charging_arm_pub = rx.Publisher(Bool, charging_arm_topic, qos_pubber)
    gps_distance_pub = rx.Publisher(Float32, "mission/gps_distance", qos_pubber)

    @rx.Subscriber(Float32, dist_to_goal_topic)
    def _dist_to_goal_cb(self, msg: Float32):
        self.bb.dist_to_station = float(msg.data)

    @rx.Subscriber(Odometry, odometry_topic)
    def _odometry_cb(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        x_target = float(self.x_switching_point)
        y_target = float(self.y_switching_point)
        distance_to_switching_point = ((x - x_target) ** 2 + (y - y_target) ** 2) ** 0.5
        self.bb.dist_to_switching_point = distance_to_switching_point
        self.get_logger().info(f"Distance to switching point: {distance_to_switching_point:.2f} m")
        self.gps_distance_pub.publish(Float32(data=float(distance_to_switching_point)))

    @rx.Subscriber(Float32, aruco_distance_topic)
    def _aruco_distance_cb(self, msg: Float32):
        distance = float(msg.data)
        if distance > 0.0:
            self.bb.aruco_distance = distance
            self.bb.charger_visible = True
        else:
            self.bb.aruco_distance = None
            self.bb.charger_visible = False

    @rx.Subscriber(String, cylinder_status_topic, qos_pubber)
    def _cylinder_status_cb(self, msg: String):
        self.bb.cylinders_visible = msg.data not in {"cylinders_lost", "idle"}

    @rx.Subscriber(Float32, cylinder_distance_topic, qos_pubber)
    def _cylinder_distance_cb(self, msg: Float32):
        self.bb.cylinder_distance = float(msg.data)
        

    @rx.Subscriber(BatteryState, battery_charging_topic, battery_qos)
    def _battery_charging_cb(self, msg: BatteryState):
        self.bb.battery_current = float(msg.current)
        self.bb.battery_voltage = float(msg.voltage)

    def on_startup(self):
        self.bb = MissionBlackboard(
            docking_exit_distance_m=float(self.docking_exit_distance_m),
            charge_start_voltage=float(self.charge_start_voltage),
            charge_done_voltage=float(self.charge_done_voltage),
            charge_voltage_confirm_s=float(self.charge_voltage_confirm_s),
        )
        self.tree = ChargingMissionTree(self.bb, self._set_charging_arm)
        self._set_charging_arm(False)
        period = 1.0 / self.tick_hz
        self.create_timer(period, self.loop)

    def _set_charging_arm(self, enabled: bool):
        self.charging_arm_pub.publish(Bool(data=enabled))

    def loop(self):
        status = self.tree.tick()
        self.active_controller_pub.publish(String(data=self.bb.active_controller))
        self.phase_pub.publish(String(data=self.bb.mission_phase))
        self.tree_status_pub.publish(String(data=status))



if __name__ == "__main__":
    bt_runner.main()
