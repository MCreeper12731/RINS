#!/usr/bin/env python3

import rclpy

from dis_homework2.srv import MoveService
from turtlesim.msg import Pose as TurtlePose

node = None
pose_publisher = None

def move_service_callback(request, response):
    global node
    global pose_publisher
    response.prev_command = request.command

    

    
    return response

def timer_callback():
    pose = TurtlePose()
    pose.x = 10.
    pose.y = 10.
    pose.theta = 0.
    pose_publisher.publish(pose)
    node.get_logger().info(f"Goal pose set to x:{pose.x}, y:{pose.y}, theta:{pose.theta}")

def main(args=None):
    global node
    global pose_publisher

    rclpy.init(args=args)

    node = rclpy.create_node("move_service_node")
    pose_publisher = node.create_publisher(TurtlePose, 'goal_pose', 10)
    server = node.create_service(MoveService, 'move_service', move_service_callback)
    timer = node.create_timer(1, timer_callback)

    node.get_logger().info("Server is ready!")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()