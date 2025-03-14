#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from dis_homework2.srv import MoveService as MoveServiceDef
from turtlesim.msg import Pose as TurtlePose
from std_msgs.msg import Bool
import random as rd

class MoveService(Node):
    def __init__(self, nodename="move_service_node"):
        super().__init__(nodename)

        self.position = None
        self.shape = ""
        self.time_left = -1
        self.points_to_visit = []
        self.index = 0

        self.pose_publisher = self.create_publisher(TurtlePose, 'goal_pose', 10)
        self.create_subscription(TurtlePose, "turtle1/pose", self.new_pose, 1)
        self.create_subscription(Bool, "goal_arrived", self.arrived, 1)
        self.create_service(MoveServiceDef, 'move_service', self.move_service_callback)

        self.create_timer(1, self.timer_callback)


    def move_service_callback(self, request, response):
        self.get_logger().info(f"Received command for {request.command}")
        self.get_logger().info(f"Time: {self.time_left}")
        response.prev_command = request.command
        self.shape = request.command
        self.time_left = request.duration
        if (self.shape == "rectangle"):
            self.points_to_visit.append([self.position[0], self.position[1], 3 * 3.141592 / 2])
            for i, [dx, dy] in enumerate([[0., 2.], [2., 2.], [2., 0.]]):
                self.points_to_visit.append([self.position[0] + dx, self.position[1] + dy, i * 3.141592 / 2])
            
        elif (self.shape == "circle"):
            for i, [dx, dy] in enumerate([[0.9951847266721969,0.0980171403295606], [0.9569403357322088,0.29028467725446233], [0.881921264348355,0.47139673682599764], [0.773010453362737,0.6343932841636455], [0.6343932841636455,0.773010453362737], [0.4713967368259978,0.8819212643483549], [0.29028467725446233,0.9569403357322089], [0.09801714032956077,0.9951847266721968], [-0.09801714032956065,0.9951847266721969], [-0.29028467725446216,0.9569403357322089], [-0.4713967368259977,0.881921264348355], [-0.6343932841636457,0.7730104533627368], [-0.773010453362737,0.6343932841636455], [-0.8819212643483552,0.4713967368259974], [-0.9569403357322088,0.2902846772544624], [-0.9951847266721969,0.0980171403295604], [-0.9951847266721969,-0.09801714032956059], [-0.9569403357322089,-0.2902846772544621], [-0.8819212643483553,-0.47139673682599725], [-0.7730104533627369,-0.6343932841636456], [-0.6343932841636459,-0.7730104533627367], [-0.4713967368259987,-0.8819212643483545], [-0.29028467725446244,-0.9569403357322088], [-0.09801714032956134,-0.9951847266721968], [0.09801714032956009,-0.9951847266721969], [0.29028467725446205,-0.9569403357322089], [0.4713967368259976,-0.881921264348355], [0.6343932841636449,-0.7730104533627374], [0.7730104533627367,-0.6343932841636459], [0.8819212643483548,-0.4713967368259979], [0.9569403357322088,-0.2902846772544625], [0.9951847266721968,-0.0980171403295614]]):
                self.points_to_visit.append([self.position[0] + dx, self.position[1] + dy, i * 2.9452])
        
        if (self.shape == "triangle"):
            self.points_to_visit.append([self.position[0], self.position[1], 4 * 3.141592 / 3])                
            for i, (dx, dy) in enumerate([[3.0, 0.0], [1.5, 0.866 * 3]]):
                self.points_to_visit.append([self.position[0] + dx, self.position[1] + dy, i * 2 * 3.141592 / 3])

        elif (self.shape == "random"):
            for _ in range(100):
                x = rd.uniform(0.5, 9.5)    # Generate a random x coordinate
                y = rd.uniform(0.5, 9.5)    # Generate a random y coordinate
                theta = rd.uniform(0, 2 * 3.141592)  # Random orientation in radians
                self.points_to_visit.append([x, y, theta])
        
        self.move()
        
        return response

    def timer_callback(self):
        # self.get_logger().info(f"Ticked once, remaining: {self.time_left}")
        if self.time_left > 0:
            self.time_left -= 1
        if self.time_left == 0:
            self.time_left = -1
            self.points_to_visit = []
            pose = TurtlePose()
            pose.x = self.position[0]
            pose.y = self.position[1]
            pose.theta = 0.
            self.pose_publisher.publish(pose)
            self.index = 0
            self.get_logger().info("Time over!")

    def new_pose(self, posemsg):
        self.position = [posemsg.x, posemsg.y, posemsg.theta]

    def arrived(self, posemsg):
        if (posemsg.data and self.time_left > 0):
            self.index += 1
            self.move()

    def move(self):
        if (len(self.points_to_visit) == 0):
            return
        pose = TurtlePose()
        pose.x = self.points_to_visit[self.index % len(self.points_to_visit)][0]
        pose.y = self.points_to_visit[self.index % len(self.points_to_visit)][1]
        pose.theta = self.points_to_visit[self.index % len(self.points_to_visit)][2]
        self.pose_publisher.publish(pose)
        self.get_logger().info(f"Drawing {self.shape} in step {self.index}")
        self.get_logger().info(f"Target: {self.points_to_visit[self.index % len(self.points_to_visit)]}")

def main(args=None):

    rclpy.init(args=args)

    ms = MoveService("MoveService")
    ms.get_logger().info("Server is ready!")
    rclpy.spin(ms)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
