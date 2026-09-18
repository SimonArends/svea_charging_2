#!/usr/bin/env python3

import csv
import time
from pathlib import Path

import svea_core.rosonic as rx

from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import ManualControl

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


class signal_logger(rx.Node):
    """
    Logs driving/steering inputs and battery measurements to a CSV file.

    ManualControl:
        y = steering input
        z = driving input

    Logged at a fixed rate, independently of the rate at which the
    input/battery messages arrive.
    """

    battery_topic = rx.Parameter("/svea_a/mavros/battery")
    drive_control_topic = rx.Parameter("/svea_a/mavros/manual_control/send")
    update_rate = rx.Parameter(5.0)

    @rx.Subscriber(ManualControl, drive_control_topic, qos_pubber)
    def manual_control_cb(self, msg):
        """
        Receive manual driving and steering inputs.

        MAVROS ManualControl:
            y = steering
            z = throttle / driving
        """
        self.steer_control = float(msg.y)
        self.drive_control = float(msg.z)

    @rx.Subscriber(BatteryState, battery_topic, qos_pubber)
    def battery_cb(self, msg: BatteryState):
        """Receive battery voltage and current."""
        self.battery_current = float(msg.current)
        self.battery_voltage = float(msg.voltage)

    def on_startup(self):
        # Latest received values.
        self.steer_control = None
        self.drive_control = None
        self.battery_current = None
        self.battery_voltage = None

        # Monotonic clock is appropriate for measuring elapsed time.
        self.start_time = time.monotonic()

        # Create output directory.
        self.log_dir = Path("/svea_ws/src/svea_logs")
        self.log_dir.mkdir(parents=True, exist_ok=True)

        # Create a unique filename based on wall-clock time.
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_path = self.log_dir / f"signals_{timestamp}.csv"

        # Open CSV file.
        self.log_file = open(
            self.log_path,
            "w",
            newline="",
            buffering=1,  # line buffered
        )

        self.csv_writer = csv.writer(self.log_file)

        # CSV header.
        self.csv_writer.writerow(
            [
                "time_since_start_s",
                "battery_voltage_V",
                "battery_current_A",
                "steering_input",
                "driving_input",
            ]
        )

        # Log at the requested rate.
        period = 1.0 / float(self.update_rate)
        self.create_timer(period, self.loop)

        self.get_logger().info(
            f"Signal logger started. Writing to {self.log_path}"
        )

    def loop(self):
        """Write the latest values to the CSV file."""
        time_since_start = time.monotonic() - self.start_time

        # # Don't write a row until all required signals have been received.
        # if (
        #     self.steer_control is None
        #     or self.drive_control is None
        #     or self.battery_current is None
        #     or self.battery_voltage is None
        # ):
        #     return

        self.csv_writer.writerow(
            [
                f"{time_since_start:.3f}",
                f"{self.battery_voltage:.3f}",
                f"{self.battery_current:.3f}",
                f"{self.steer_control:.6f}",
                f"{self.drive_control:.6f}",
            ]
        )

    def on_shutdown(self):
        """Close the log file cleanly."""
        if hasattr(self, "log_file") and self.log_file:
            self.log_file.flush()
            self.log_file.close()

        self.get_logger().info("Signal logger stopped.")


if __name__ == "__main__":
    signal_logger.main()