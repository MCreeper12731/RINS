#!/usr/bin/env python3

import rclpy
import time
import random

from dis_homework1.srv import CustomService

customnode = None

def main(args=None):
    global customnode

    rclpy.init(args=args)

    customnode = rclpy.create_node("custom_client_node")
    client = customnode.create_client(CustomService, 'custom_service')

    request = CustomService.Request()
    
    while rclpy.ok():
        request.field1 = "CusToMSerVicE"
        request.field2 = [random.randint(42,69) for x in range(10)]
        
        customnode.get_logger().info("Sending a custom request!")
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(customnode, future)
        response = future.result()
        customnode.get_logger().info('Result of custom_service: for %s %s: %s %s' %(request.field1, request.field2, response.field3, response.field4))
        
        time.sleep(1)

    customnode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
