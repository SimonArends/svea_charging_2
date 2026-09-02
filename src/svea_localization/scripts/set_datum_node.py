#!/usr/bin/env python3

"""Set a fixed datum on robot_localization's navsat_transform_node."""

import ast
import os

import rclpy
import yaml
from geographic_msgs.msg import GeoPose
from rclpy.node import Node
from robot_localization.srv import SetDatum
from tf_transformations import quaternion_from_euler


class SetDatumNode(Node):
    def __init__(self):
        super().__init__("set_datum_node")

        self.declare_parameter("datum_service", "datum")
        self.declare_parameter("datum_file", "")
        self.declare_parameter("datum_data", "[]")
        self.declare_parameter("service_timeout", 60.0)

        self.datum_service = str(self.get_parameter("datum_service").value)
        self.datum_file = str(self.get_parameter("datum_file").value)
        self.datum_data = str(self.get_parameter("datum_data").value)
        self.service_timeout = float(self.get_parameter("service_timeout").value)
        self.datum = self._load_datum()
        self.client = self.create_client(SetDatum, self.datum_service)

    def _load_datum(self):
        if self.datum_file:
            if not os.path.isfile(self.datum_file):
                raise FileNotFoundError(f"Datum file does not exist: {self.datum_file}")
            with open(self.datum_file, "r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream)
            datum = data.get("datum") if isinstance(data, dict) else None
            if not isinstance(datum, dict):
                raise ValueError("Datum YAML must contain a 'datum' mapping")
        else:
            values = ast.literal_eval(self.datum_data)
            if not isinstance(values, (list, tuple)) or len(values) != 3:
                raise ValueError("datum_data must be [latitude, longitude, yaw]")
            datum = {"latitude": values[0], "longitude": values[1], "yaw": values[2]}

        return {
            "latitude": float(datum["latitude"]),
            "longitude": float(datum["longitude"]),
            "yaw": float(datum["yaw"]),
        }

    def set_datum(self):
        self.get_logger().info(
            f"Waiting up to {self.service_timeout:.1f}s for {self.datum_service}"
        )
        if not self.client.wait_for_service(timeout_sec=self.service_timeout):
            raise TimeoutError(f"Datum service unavailable: {self.datum_service}")

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.datum["yaw"])
        request = SetDatum.Request()
        request.geo_pose = GeoPose()
        request.geo_pose.position.latitude = self.datum["latitude"]
        request.geo_pose.position.longitude = self.datum["longitude"]
        request.geo_pose.position.altitude = 0.0
        request.geo_pose.orientation.x = qx
        request.geo_pose.orientation.y = qy
        request.geo_pose.orientation.z = qz
        request.geo_pose.orientation.w = qw

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout)
        if not future.done() or future.result() is None:
            raise TimeoutError("SetDatum request timed out or failed")
        if hasattr(future.result(), "success") and not future.result().success:
            raise RuntimeError("navsat_transform_node rejected the datum")

        self.get_logger().info(
            "Datum set successfully: "
            f"lat={self.datum['latitude']:.7f}, "
            f"lon={self.datum['longitude']:.7f}, yaw={self.datum['yaw']:.6f} rad"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SetDatumNode()
        node.set_datum()
    except Exception as ex:
        if node is not None:
            node.get_logger().fatal(str(ex))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
