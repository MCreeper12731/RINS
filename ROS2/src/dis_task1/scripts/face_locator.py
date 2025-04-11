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

from dis_task1.msg import MoverMessage

import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped

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

        self.goal_publisher = self.create_publisher(MoverMessage, "/mover_command", QoSReliabilityPolicy.BEST_EFFORT)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    
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
    
    def face_found_callback(self, data: Marker):
    # If we don't have a valid robot pose already, skip:
        if self.position is None:
            self.get_logger().info("Position is None")
            return

        # Create a PoseStamped for the marker (which is in base_link)
        pose_stamped = PoseStamped()
        pose_stamped.header = data.header
        pose_stamped.pose = data.pose

        # Try to transform the marker pose from base_link to map using the marker’s stamp
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",        
                data.header.frame_id[1::],  
                data.header.stamp,    
                rclpy.duration.Duration(seconds=1.0)
            )
            # Transform the pose into map frame
            pose_in_map = do_transform_pose(pose_stamped.pose, transform)
        except Exception as e:
            self.get_logger().error("TF transform error: " + str(e))
            return

        # Now, pose_in_map contains the face location in the map frame
        face_abs_pos = pose_in_map.position
        transformed_orientation = pose_in_map.orientation

        # Optionally, you can log the transformed pose:
        self.get_logger().info(
            f"Transformed face position: ({face_abs_pos.x:.2f}, {face_abs_pos.y:.2f}, {face_abs_pos.z:.2f})"
        )

        # Check for duplicates and call goto_face if this is a new detection:
        if not self.check_detected(face_abs_pos):
            self.faces.append(face_abs_pos)
            self.goto_face(face_abs_pos, orientation=transformed_orientation)


        if len(self.faces) > 10:
            self.faces.pop(0)

                

    def goto_face(self, face_point: Point, orientation: Quaternion):
        
        ms = MoverMessage()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = face_point.x
        goal_pose.pose.position.y = face_point.y
        goal_pose.pose.position.z = face_point.z
        goal_pose.pose.orientation = orientation

        ms.location = goal_pose
        ms.type = "face"
        ms.data = ""

        self.goal_publisher.publish(ms)

        self.get_logger().info(
            f"Sending goal: pos=({face_point.x}, {face_point.y}, {face_point.z}), "
            f"orientation=({orientation.x}, {orientation.y}, {orientation.z}, {orientation.w})"
        )
        

    
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
    
    def yaw_to_quaternion(yaw: float) -> Quaternion:
    # Create a quaternion from a yaw angle (roll and pitch are zero)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)


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