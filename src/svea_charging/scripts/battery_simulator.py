#!/usr/bin/env python3

import math
import time

import svea_core.rosonic as rx

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import ManualControl


class battery_simulator(rx.Node):

    # Parameters
    battery_empty_voltage = rx.Parameter(10.0)
    battery_full_voltage = rx.Parameter(12.6)
    # 900 mAh = 0.9 Ah
    battery_capacity_ah = rx.Parameter(0.9)
    battery_charge_current = rx.Parameter(9.0)
    battery_discharge_current_stationary = rx.Parameter(-0.9)
    battery_discharge_current_driving = rx.Parameter(-1.8)

    battery_charging_topic = rx.Parameter("/self/mavros/battery")
    odometry_topic = rx.Parameter("odometry/local")
    drive_contol_topic = rx.Parameter("mavros/manual_control/send")

    # Charging station position
    charging_x = rx.Parameter(4.52)
    charging_y = rx.Parameter(-1.69)
    charging_radius = rx.Parameter(0.025)

    # Battery update rate [Hz]
    update_rate = rx.Parameter(10.0)

    # Publishers
    battery_pub = rx.Publisher(BatteryState, battery_charging_topic)

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

    # Subscribers
    @rx.Subscriber(Odometry, odometry_topic)
    def odometry_cb(self, msg):
        """ Update the robot position and determine whether it is inside the charging area."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

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
        # Calculate distance to charging station
        dx = self.robot_x - self.charging_x
        dy = self.robot_y - self.charging_y
        distance = math.sqrt(dx * dx + dy * dy)
        is_charging = distance <= self.charging_radius

        # Determine current
        if is_charging:
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


if __name__ == "__main__":
    battery_simulator.main()