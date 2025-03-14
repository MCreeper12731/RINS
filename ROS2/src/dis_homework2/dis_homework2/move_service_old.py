#!/usr/bin/env python3

import rclpy

from dis_homework2.srv import MoveService
from turtlesim.msg import Pose as TurtlePose
from std_msgs.msg import Bool

node = None
pose_publisher = None
position = None
shape = ""
time_left = 0
points_to_visit = []
index = 0

def move_service_callback(request, response):
    global node
    global pose_publisher
    global points_to_visit
    global index
    global shape
    global time_left
    response.prev_command = request.command
    shape = request.command
    time_left = request.duration
    if (shape == "rectangle"):
        points_to_visit.append(position)
        for dx, dy in [[0., 1.], [1., 1.], [1., 0.]]:
            points_to_visit.append([position[0] - dx, position[1] - dy])
    return response

def timer_callback():
    global time_left
    global points_to_visit
    time_left -= 1
    if time_left < 0:
        time_left = 0
        points_to_visit = []
        pose = TurtlePose()
        pose.x = position[0]
        pose.y = position[1]
        pose.theta = 0.
        pose_publisher.publish(pose)
        node.get_logger().info("Finished!")

def new_pose(posemsg):
    global position
    position = [posemsg.x, posemsg.y, posemsg.theta]



def arrived(posemsg):
    global index
    if (posemsg.data):
        index += 1
        node.get_logger().info(f"New index set to {index}")
        if (len(points_to_visit) == 0):
            return
        pose = TurtlePose()
        pose.x = points_to_visit[index % len(points_to_visit)][0]
        pose.y = points_to_visit[index % len(points_to_visit)][1]
        pose.theta = 0.
        pose_publisher.publish(pose)
        node.get_logger().info(f"Drawing {shape} in step {index}")

def main(args=None):
    global node
    global pose_publisher

    rclpy.init(args=args)

    node = rclpy.create_node("move_service_node")
    pose_publisher = node.create_publisher(TurtlePose, 'goal_pose', 10)
    node.create_subscription(TurtlePose, "turtle1/pose", new_pose, 1)
    node.create_subscription(Bool, "goal_arrived", arrived, 1)
    node.create_service(MoveService, 'move_service', move_service_callback)
    timer = node.create_timer(1, timer_callback)

    node.get_logger().info("Server is ready!")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
