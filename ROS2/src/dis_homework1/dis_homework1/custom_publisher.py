#!/usr/bin/env python3

#print('I am alive!')
import rclpy
import time

from dis_homework1.msg import CustomMessage

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("custom_publisher_node")

    publisher = node.create_publisher(CustomMessage, "/custom_topic", 10)

    message = CustomMessage()
    message.field1 = "Field1"
    message.field2 = 2
    message.field3 = True


    while rclpy.ok():
        publisher.publish(message)
        message.field2 += 1

        node.get_logger().info("Publisher: I performed one iteration!")
        time.sleep(1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
