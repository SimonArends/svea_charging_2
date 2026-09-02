#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

# Importiamo la funzione utility dal progetto SVEA
from svea_core.utils.viz_util import publish_lidar_rays


class LidarRayPublisher(Node):
    def __init__(self):
        super().__init__('lidar_ray_publisher')

        self.rays_pub = self.create_publisher(Marker, '/lidar_rays', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/self/scan',
            self.scan_callback,
            10
        )

        self.lidar_pos = [0.0, 0.0]

    def scan_callback(self, scan_msg):

        ranges = scan_msg.ranges
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment

        points = []

        for i, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r) or r <= 0.0 or r > 1.2:
                continue

            angle = angle_min + (i * angle_increment)
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append([x, y])

        publish_lidar_rays(self.rays_pub, self.lidar_pos, points)


def main(args=None):
    rclpy.init(args=args)
    node = LidarRayPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()