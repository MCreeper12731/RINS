#!/usr/bin/env python3

import rclpy
import time

from dis_homework1.srv import CustomService

customnode = None

def custom_service_callback(request, response):
    global customnode
    response.field3 = f"[{request.field1}] the sum is"
    response.field4 = int(sum(request.field2))
    customnode.get_logger().info('Incoming request\na: %s b: %s' % (request.field1, request.field2))
    return response

def main(args=None):
    global customnode

    rclpy.init(args=args)

    customnode = rclpy.create_node("custom_server_node")
    server = customnode.create_service(CustomService, 'custom_service', custom_service_callback)

    customnode.get_logger().info("Server is ready!")
    rclpy.spin(customnode)

    customnode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
