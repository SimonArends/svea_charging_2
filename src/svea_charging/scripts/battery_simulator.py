#!/usr/bin/env python3

import math
import time

import svea_core.rosonic as rx

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Bool, Float32, String

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class battery_simulator(rx.Node):

    # Parameters
    battery_empty_voltage = rx.Parameter(10.0)
    battery_full_voltage = rx.Parameter(12.6)
    # 900 mAh = 0.9 Ah
    battery_capacity_ah = rx.Parameter(0.9)
    battery_charge_current = rx.Parameter(19.0)
    battery_discharge_current_stationary = rx.Parameter(-0.9)
    battery_discharge_current_driving = rx.Parameter(-1.8)

    battery_charging_topic = rx.Parameter("/self/mavros/battery")
    odometry_topic = rx.Parameter("odometry/local")
    drive_contol_topic = rx.Parameter("mavros/manual_control/send")
    docking_status_topic = rx.Parameter("cylinder_docking/velocity_phase")
    charging_status_topic = rx.Parameter("/self/mission/charging_status")
    mission_phase_topic = rx.Parameter("/self/mission/phase")
    parking_location_topic = rx.Parameter("/outdoor_stanley/parking_location")

    # Battery update rate [Hz]
    update_rate = rx.Parameter(10.0)

    # Publishers
    battery_pub = rx.Publisher(BatteryState, battery_charging_topic)
    parking_pub = rx.Publisher(String, parking_location_topic)

    # State
    def on_startup(self):
        self.battery_soc = 0.95
        # Robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        # Manual control
        self.manual_control_z = 500
        # Timing
        self.last_update_time = time.monotonic()
        self.docking_status = False
        self.charging_status = False
        self.charging_stopped = False
        self.mission_phase = None
        self.parking_location = "B"

    @rx.Subscriber(String, docking_status_topic, qos_pubber)
    def docking_status_cb(self, msg: String):
        """ Update the docking status. """
        message = msg.data
        self.docking_status = message in {"docked"}

    @rx.Subscriber(Bool, charging_status_topic, qos_pubber)
    def charging_status_cb(self, msg: Bool):
        """ Update the charging status. """
        was_charging = self.charging_status
        self.charging_status = msg.data
        is_charging = self.charging_status
        if was_charging and not is_charging:
            self.get_logger().info("Charging stopped.")
            self.charging_stopped = True
    
    @rx.Subscriber(String, mission_phase_topic, qos_pubber)
    def mission_phase_cb(self, msg: String):
        """ Update the mission phase. """
        self.mission_phase = msg.data

    @rx.Subscriber(ManualControl, drive_contol_topic)
    def manual_control_cb(self, msg):
        """
        Determine whether the robot is moving.
            z < 490 -> moving
        """
        self.manual_control_z = msg.z

    # Battery simulation
    @rx.Timer(0.1)
    def update_battery(self):
        """ Update battery state and publish BatteryState. """
        # Determine current
        if self.docking_status and (self.mission_phase == "docking" or self.mission_phase == "docked" or self.mission_phase == "charging"):
            current = self.battery_charge_current
        elif self.manual_control_z < 490:
            current = self.battery_discharge_current_driving
        else:
            current = self.battery_discharge_current_stationary

        # Calculate elapsed time
        now = time.monotonic()
        dt = now - self.last_update_time
        self.last_update_time = now

        # Update state of charge
        # current [A]
        # dt      [s]
        # capacity [Ah]
        # dSOC = I * dt / (capacity * 3600)
        # Positive current -> charging
        # Negative current -> discharging

        self.battery_soc += (current * dt / (self.battery_capacity_ah * 3600.0))

        # Clamp SOC to [0, 1]
        self.battery_soc = max(0.0, min(1.0, self.battery_soc))

        # Calculate voltage
        # Linear relationship:
        # SOC = 0 -> 10.0 V
        # SOC = 1 -> 12.6 V

        voltage = (self.battery_empty_voltage + self.battery_soc * (self.battery_full_voltage - self.battery_empty_voltage))

        # Create BatteryState message
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(voltage)
        msg.current = float(current)
        msg.percentage = float(self.battery_soc)
        
        # Publish
        self.battery_pub.publish(msg)
        self.parking_pub.publish(String(data=str(self.parking_location)))


if __name__ == "__main__":
    battery_simulator.main()