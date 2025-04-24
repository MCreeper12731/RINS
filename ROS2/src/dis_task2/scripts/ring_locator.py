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

import time
from cv_bridge import CvBridge


class RingLocator(Node):
    def __init__(self):
        super().__init__("ring_locator")

        self.delta = 1.
        self.color_delta = 50
        self.position = None
        self.yaw = 0.0
        self.rings = []
        self.create_subscription(Marker, "/ring_marker", self.ring_found_callback, QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.position_callback, QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Bool, "/ring_locator/start", self.start_tour_callback, QoSReliabilityPolicy.BEST_EFFORT)
        self.marker_publisher = self.create_publisher(Marker, "/debug_marker", QoSReliabilityPolicy.BEST_EFFORT)
    
        self.goal_publisher = self.create_publisher(MoverMessage, "/mover_command", QoSReliabilityPolicy.BEST_EFFORT)

        self.create_timer(0.5, self.timer_callback)

        self.bridge = CvBridge()

    def start_tour_callback(self, data : Bool):
        self.get_logger().info("STARTING TOUR")
        rings = []
        file = open("../ring_locations.txt", "r")
        for line in file.readlines():
            x, y, z = line.split()
            rings.append(Point(x=float(x), y=float(y), z=float(z)))
        # file.close()
        while True:
            for ring in rings:
                
                rc = RobotCommander()

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = rc.get_clock().now().to_msg()

                goal_pose.pose.position.x = ring.x
                goal_pose.pose.position.y = ring.y
                goal_pose.pose.orientation = rc.YawToQuaternion(0.57)

                rc.goToPose(goal_pose)

                while not rc.isTaskComplete():
                    self.get_logger().info("Waiting for the task to complete...")
                    time.sleep(1)

    
    def ring_found_callback(self, data : Marker):
        if (self.position is None):
            #self.get_logger().info("Position is None")
            return
        #print("checking new ring")

        ring_rel_pos = RingLocator.rotate_vector(data.pose.position, self.yaw)
        color_word = self.classify_color(data.color.r, data.color.g, data.color.b)

        ring_abs_pos = Point(x=ring_rel_pos.x + self.position.x, y=ring_rel_pos.y + self.position.y, z=ring_rel_pos.z + self.position.z)
        ring = {
            "pos": ring_abs_pos,
            "color": (data.color.r, data.color.g, data.color.b),
            "color_word": color_word.lower()
        }
        if not self.check_detected(ring):
            self.rings.append(ring)
            self.get_logger().info(f"Ring info: Position: {ring['pos']}, RGB: {ring['color']}, Likely color: {ring['color_word']}, Number of rings: {len(self.rings)}")
            #with open("../ring_locations.txt", "w") as self.rings_file:
            #    for ring in self.rings:
            #        self.rings_file.write(f"{ring.x} {ring.y} {ring.z}\n")
            #self.goto_ring(ring_abs_pos)
            self.goto_ring(ring)
            
    

        if len(self.rings) > 10:
            self.rings.pop(0)
    
    def goto_ring(self, ring):
        
        ms = MoverMessage()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = ring["pos"].x
        goal_pose.pose.position.y = ring["pos"].y
        goal_pose.pose.position.z = ring["pos"].z
        goal_pose.pose.orientation = Quaternion(x=0., y=0., z=0., w=1.)

        ms.location = goal_pose
        ms.type = "ring"
        ms.data = ring["color_word"]

        self.goal_publisher.publish(ms)

    def position_callback(self, data : PoseWithCovarianceStamped):
        self.position = data.pose.pose.position
        self.yaw = RingLocator.quaternion_to_yaw(data.pose.pose.orientation)
        
    def post_markers(self):
        
        i = -1
        for full_ring in self.rings:
            ring = full_ring["pos"]
            i += 1
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
            marker.pose.position.x = float(ring.x)
            marker.pose.position.y = float(ring.y)
            marker.pose.position.z = float(ring.z)

            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

            self.marker_publisher.publish(marker)
            #print("RING COLOR:", color)

    def timer_callback(self):
        self.post_markers()
        # self.get_logger().info(f"{[(el.x, el.y, el.z) for el in self.rings]}")
    
    def check_detected(self, point):
        new_color = point["color"]
        i = -1
        for ring in self.rings:
            f = ring["pos"]
            i += 1
            dist = self.distance(f, point["pos"])
            # check distance difference
            if dist < self.delta:
                # Check color difference
                color_diff = np.linalg.norm(np.array(ring["color"]) - np.array(new_color))
                if(f == point["pos"]): # if the pos is the exact same
                    return True
                
                if color_diff < self.color_delta or ring["color_word"] == point["color_word"]:
                    self.rings[i]["pos"].x = (f.x + point["pos"].x) / 2.0
                    self.rings[i]["pos"].y = (f.y + point["pos"].y) / 2.0
                    self.rings[i]["pos"].z = (f.z + point["pos"].z) / 2.0
                    #print(f"Difference is '{dist}' with delta '{self.delta}' \nnew ring: {point}\nold ring: {ring}")
                    return True
                else:
                    # Too different in color — treat as new ring
                    #print("TOO DIFFERENT IN COLOR. ADDING...", dist)
                    continue
        return False

    
    # euclidean distance
    def distance(self,p1,p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y- p2.y)**2 + (p1.z - p2.z)**2)
    
    def classify_color(self, r, g, b):
        # Define thresholds for classification
        if r < 50 and g < 50 and b < 50:
            return "BLACK"
        elif r > 200 and g > 200 and b > 200:
            return "WHITE"
        
        # Check if the green channel is the dominant channel
        elif g > r and g > b:
            # If the green is dominant but the values are not too low (light green)
            if g > 150:
                return "GREEN"
            else:
                return "YELLOW"  # Light Green shades might get misclassified here, adjust thresholds as needed

        # Check for Yellow: High red + high green, low blue
        elif r > 150 and g > 150 and b < 100:
            return "YELLOW"
        
        # Check for Orange: High red, medium green, low blue
        elif r > 150 and g > 100 and b < 100:
            return "ORANGE"

        # Check for Red: High red, low green and blue
        elif r > g and r > b:
            return "RED"
        
        # Check for Blue: High blue, low red and green
        elif b > r and b > g:
            return "BLUE"

        else:
            return "Unknown"  # For unexpected cases


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
    print('Ring detector starting.')
    
    rclpy.init(args=args)
    ring_locator = RingLocator()

    rclpy.spin(ring_locator)

    ring_locator.destroyNode()

if __name__=="__main__":
    main()