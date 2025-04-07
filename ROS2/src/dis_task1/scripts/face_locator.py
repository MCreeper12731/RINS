#!/usr/bin/env python3

import rclpy
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from lifecycle_msgs.srv import GetState
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from std_msgs.msg import Bool
import numpy as np
from robot_commander import RobotCommander

import time

class FaceLocator(Node):
    def __init__(self):
        super().__init__("face_locator")

        self.delta = 1.
        self.position = None
        self.yaw = 0.0
        self.faces : list[Point] = []
        self.create_subscription(Marker, "/people_marker", self.face_found_callback, QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.position_callback, QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Bool, "/face_locator/start", self.start_tour_callback, QoSReliabilityPolicy.BEST_EFFORT)
        self.marker_publisher = self.create_publisher(Marker, "/debug_marker", QoSReliabilityPolicy.BEST_EFFORT)
    
        self.create_timer(0.5, self.timer_callback)

    def start_tour_callback(self, data : Bool):
        self.get_logger().info("YAY")
        faces : list[Point] = []
        file = open("../face_locations.txt", "r")
        for line in file.readlines():
            x, y, z = line.split()
            faces.append(Point(x=float(x), y=float(y), z=float(z)))
        # file.close()
        while True:
            for face in faces:
                
                rc = RobotCommander()

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = rc.get_clock().now().to_msg()

                goal_pose.pose.position.x = face.x
                goal_pose.pose.position.y = face.y
                goal_pose.pose.orientation = rc.YawToQuaternion(0.57)

                rc.goToPose(goal_pose)

                while not rc.isTaskComplete():
                    self.get_logger().info("Waiting for the task to complete...")
                    time.sleep(1)
    
    def face_found_callback(self, data : Marker):
        if (self.position is None):
            self.get_logger().info("Position is None")
            return

        face_rel_pos = FaceLocator.rotate_vector(data.pose.position, self.yaw)

        face_abs_pos = Point(x=face_rel_pos.x + self.position.x, y=face_rel_pos.y + self.position.y, z=face_rel_pos.z + self.position.z)

        if not self.check_detected(face_abs_pos):
            self.faces.append(face_abs_pos)
            self.get_logger().info(str(len(self.faces)))
            #with open("../face_locations.txt", "w") as self.faces_file:
            #    for face in self.faces:
            #        self.faces_file.write(f"{face.x} {face.y} {face.z}\n")
            self.goto_face(face_abs_pos)
            
    

        if len(self.faces) > 10:
            self.faces.pop(0)
    
    def position_callback(self, data : PoseWithCovarianceStamped):
        self.position = data.pose.pose.position
        self.yaw = FaceLocator.quaternion_to_yaw(data.pose.pose.orientation)
        
    def post_markers(self):
        
        for i, face in enumerate(self.faces):
            marker = Marker()

            marker.header.frame_id = "map"
            # marker.header.stamp = rospy.get_rostime()

            marker.type = 2
            marker.id = i

            # Set the scale of the marker
            scale = 0.5
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale

            # Set the color
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Set the pose of the marker
            marker.pose.position.x = float(face.x)
            marker.pose.position.y = float(face.y)
            marker.pose.position.z = float(face.z)

            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

            self.marker_publisher.publish(marker)

    def timer_callback(self):
        self.post_markers()
        # self.get_logger().info(f"{[(el.x, el.y, el.z) for el in self.faces]}")
    
    def check_detected(self, point):
        for i, f in enumerate(self.faces):
            if self.distance(f, point) < self.delta:
                self.faces[i].x = (f.x + point.x) / 2.0
                self.faces[i].y = (f.y + point.y) / 2.0
                self.faces[i].z = (f.z + point.z) / 2.0
                return True
        return False

    
    # euclidean distance
    def distance(self,p1,p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y- p2.y)**2 + (p1.z - p2.z)**2)


    def quaternion_to_yaw(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def rotate_vector(vector, theta) -> Point:
        R = np.array([
            [math.cos(theta), -math.sin(theta), 0],
            [math.sin(theta),  math.cos(theta), 0],
            [0,                0,               1]
        ])
        new_vector = R.dot(np.array([vector.x, vector.y, vector.z]))
        return Point(x=new_vector[0], y=new_vector[1], z=new_vector[2])

def main(args=None):
    
    rclpy.init(args=args)
    face_locator = FaceLocator()

    rclpy.spin(face_locator)

    face_locator.destroyNode()

if __name__=="__main__":
    main()