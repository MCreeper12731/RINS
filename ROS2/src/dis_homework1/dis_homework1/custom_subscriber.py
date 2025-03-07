#!/usr/bin/env python3

import rclpy

from dis_homework1.msg import CustomMessage

mynode = None

def topic_callback(msg):
    global mynode
    mynode.get_logger().info('I heard: "%s, %d, %s"' % (msg.field1, msg.field2, msg.field3))

def main(args=None):
    global mynode
    rclpy.init(args=args)
    mynode = rclpy.create_node("custom_subscriber_node")
    
    subscription = mynode.create_subscription(CustomMessage, "/custom_topic", topic_callback, 10)

    while rclpy.ok():
        rclpy.spin_once(mynode)

    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
