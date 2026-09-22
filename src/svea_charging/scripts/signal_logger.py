#!/usr/bin/env python3

import csv
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import NavSatFix


# Battery QoS
qos_bat = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class SignalLogger(Node):

    def __init__(self):
        super().__init__("signal_logger")

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter(
            "battery_topic",
            "/svea_a/mavros/battery",
        )

        self.declare_parameter(
            "wheel_odom_topic",
            "/svea_a/mavros/wheel_odometry/velocity",
        )

        self.declare_parameter(
            "gps_topic",
            "/svea_a/gps/fix",
        )

        self.declare_parameter(
            "update_rate",
            0.2,
        )

        battery_topic = self.get_parameter(
            "battery_topic"
        ).value

        wheel_odom_topic = self.get_parameter(
            "wheel_odom_topic"
        ).value

        gps_topic = self.get_parameter(
            "gps_topic"
        ).value

        update_rate = float(
            self.get_parameter("update_rate").value
        )

        # ---------------------------------------------------------
        # Latest received values
        # ---------------------------------------------------------

        self.battery_current = None
        self.battery_voltage = None
        self.velocity = None

        self.latitude = None
        self.longitude = None

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------

        self.battery_sub = self.create_subscription(
            BatteryState,
            battery_topic,
            self.battery_cb,
            qos_bat,
        )

        self.velocity_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            wheel_odom_topic,
            self.velocity_cb,
            qos_profile_sensor_data,
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            gps_topic,
            self.gps_cb,
            qos_profile_sensor_data,
        )

        # ---------------------------------------------------------
        # Timing
        # ---------------------------------------------------------

        self.start_time = time.monotonic()

        period = 1.0 / update_rate

        self.timer = self.create_timer(
            period,
            self.loop,
        )

        # ---------------------------------------------------------
        # CSV setup
        # ---------------------------------------------------------

        self.log_dir = Path("/svea_ws/src/svea_logs")
        self.log_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        timestamp = time.strftime("%Y%m%d_%H%M%S")

        self.log_path = (
            self.log_dir
            / f"signals_{timestamp}.csv"
        )

        self.log_file = open(
            self.log_path,
            "w",
            newline="",
            buffering=1,
        )

        self.csv_writer = csv.writer(
            self.log_file
        )

        self.csv_writer.writerow(
            [
                "time_since_start_s",
                "battery_voltage_V",
                "battery_current_A",
                "velocity_mps",
                "latitude_deg",
                "longitude_deg",
            ]
        )

        # ---------------------------------------------------------
        # Startup logging
        # ---------------------------------------------------------

        self.get_logger().info(
            f"Signal logger started. "
            f"Writing to {self.log_path}"
        )

        self.get_logger().info(
            f"Battery topic: {battery_topic}"
        )

        self.get_logger().info(
            f"Wheel odometry topic: {wheel_odom_topic}"
        )

        self.get_logger().info(
            f"GPS topic: {gps_topic}"
        )

    # -------------------------------------------------------------
    # Battery callback
    # -------------------------------------------------------------

    def battery_cb(self, msg: BatteryState):
        self.battery_current = float(msg.current)
        self.battery_voltage = float(msg.voltage)

    # -------------------------------------------------------------
    # Velocity callback
    # -------------------------------------------------------------

    def velocity_cb(
        self,
        msg: TwistWithCovarianceStamped,
    ):
        self.velocity = float(
            msg.twist.twist.linear.x
        )

    # -------------------------------------------------------------
    # GPS callback
    # -------------------------------------------------------------

    def gps_cb(self, msg: NavSatFix):
        self.latitude = float(msg.latitude)
        self.longitude = float(msg.longitude)

    # -------------------------------------------------------------
    # Logging loop
    # -------------------------------------------------------------

    def loop(self):

        time_since_start = (
            time.monotonic() - self.start_time
        )

        self.csv_writer.writerow(
            [
                f"{time_since_start:.3f}",

                "" if self.battery_voltage is None
                else f"{self.battery_voltage:.3f}",

                "" if self.battery_current is None
                else f"{self.battery_current:.3f}",

                "" if self.velocity is None
                else f"{self.velocity:.6f}",

                "" if self.latitude is None
                else f"{self.latitude:.8f}",

                "" if self.longitude is None
                else f"{self.longitude:.8f}",
            ]
        )

    # -------------------------------------------------------------
    # Shutdown
    # -------------------------------------------------------------

    def destroy_node(self):

        self.get_logger().info(
            "Signal logger stopped."
        )

        if hasattr(self, "log_file"):
            self.log_file.flush()
            self.log_file.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = SignalLogger()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()