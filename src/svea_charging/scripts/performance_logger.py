#!/usr/bin/env python3
import time

import svea_core.rosonic as rx

from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

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


class performance_logger(rx.Node):
    # Current above this value is treated as actual charging.
    charging_detect_current = rx.Parameter(-0.7)

    # Control loop rate [Hz]
    update_rate = rx.Parameter(20.0)

    # Minimum time between printed performance log lines [s]
    print_interval = rx.Parameter(60.0)

    battery_charging_topic_a = rx.Parameter("/svea_a/mavros/battery")
    battery_charging_topic_b = rx.Parameter("/svea_b/mavros/battery")

    # --- Publisher -------------------------------------------------
    performance_pub = rx.Publisher(String, "/mission/performance", qos_pubber)

    # --- Subscribers -----------------------------------------------
    @rx.Subscriber(BatteryState, battery_charging_topic_a, qos_pubber)
    def _battery_charging_cb_a(self, msg: BatteryState):
        self.battery_current_a = float(msg.current)

    @rx.Subscriber(BatteryState, battery_charging_topic_b, qos_pubber)
    def _battery_charging_cb_b(self, msg: BatteryState):
        self.battery_current_b = float(msg.current)

    @rx.Subscriber(String, "/svea_a/mission/active_controller")
    def _mission_active_a_cb(self, msg: String):
        self.active_controller_a = msg.data

    @rx.Subscriber(String, "/svea_b/mission/active_controller")
    def _mission_active_b_cb(self, msg: String):
        self.active_controller_b = msg.data

    def on_startup(self):
        self.battery_current_a = None
        self.battery_current_b = None
        self.active_controller_a = None
        self.active_controller_b = None

        # Accumulated time spent in each state [s]
        self.transport_time_a = 0.0
        self.transport_time_b = 0.0
        self.charger_active_time = 0.0

        # Time reference for the next calculation
        self.last_update_time = time.monotonic()

        # Time at which performance was last printed
        self.last_print_time = self.last_update_time

        # Total time since the logger started
        self.total_time = 0.0

        period = 1.0 / self.update_rate
        self.create_timer(period, self.loop)

    def loop(self):
        now = time.monotonic()
        dt = now - self.last_update_time
        self.last_update_time = now

        # Protect against unexpected negative/very large time steps.
        if dt < 0.0:
            dt = 0.0

        self.total_time += dt


        # Determine whether each SVEA is currently performing a transport mission.
        svea_a_transporting = (
            self.active_controller_a == "transport_stanley"
        )

        svea_b_transporting = (
            self.active_controller_b == "transport_stanley"
        )

        # Determine whether either SVEA is currently charging.

        # A SVEA is charging when its measured current is above the
        # charging detection threshold.
        svea_a_charging = (
            self.battery_current_a is not None
            and self.battery_current_a > self.charging_detect_current
        )

        svea_b_charging = (
            self.battery_current_b is not None
            and self.battery_current_b > self.charging_detect_current
        )

        charger_active = svea_a_charging or svea_b_charging

        # Accumulate time spent in each state.
        if svea_a_transporting:
            self.transport_time_a += dt

        if svea_b_transporting:
            self.transport_time_b += dt

        if charger_active:
            self.charger_active_time += dt

        # Calculate percentages.
        if self.total_time > 0.0:
            transport_percentage_a = (
                100.0 * self.transport_time_a / self.total_time
            )

            transport_percentage_b = (
                100.0 * self.transport_time_b / self.total_time
            )

            charger_utilization_percentage = (
                100.0 * self.charger_active_time / self.total_time
            )
        else:
            transport_percentage_a = 0.0
            transport_percentage_b = 0.0
            charger_utilization_percentage = 0.0

        # Construct and publish the performance message.
        message = (
            f"Svea_a transport time percentage: "
            f"{transport_percentage_a:.2f}%, "
            f"Svea_b transport time percentage: "
            f"{transport_percentage_b:.2f}%, "
            f"Charger utilization time percentage: "
            f"{charger_utilization_percentage:.2f}%"
        )

        self.performance_pub.publish(String(data=message))

        # Print periodically.
        if now - self.last_print_time >= self.print_interval:
            self.get_logger().info(message)
            self.last_print_time = now


if __name__ == "__main__":
    performance_logger.main()