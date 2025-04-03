#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class Sweeper(Node):
    def __init__(self):
        super().__init__("sweeper")

        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10
        )

        self.get_logger().info("Sweeper constructed!")


    def map_callback(self, grid : OccupancyGrid):
        self.get_logger().info(grid.data)

def main():
    rclpy.init(args=None)
    node = Sweeper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()