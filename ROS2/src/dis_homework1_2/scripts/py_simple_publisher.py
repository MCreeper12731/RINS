#!/usr/bin/env python3

#print('I am alive!')
import rclpy
import time

from r"../msg" import String

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("py_CustomPublisher_node")
    
    publisher = node.create_publisher(String, "/CustomPublisher", 10)
    
    message = String()
    message_same = "Hello world of ROS2 publishers! This is message number: "
    message_num = 3

    while rclpy.ok():
        message.data = message_same + str(message_num)
        publisher.publish(message)
        message_num += 1

        node.get_logger().info("Publisher: I performed one iteration!")
        time.sleep(1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
