#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from visualization_msgs.msg import Marker

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from ultralytics import YOLO

import time

# from rclpy.parameter import Parameter
# from rcl_interfaces.msg import SetParametersResult

class detect_faces(Node):

	def __init__(self):
		super().__init__('detect_faces')

		self.declare_parameters(
			namespace='',
			parameters=[
				('device', ''),
		])

		marker_topic = "/people_marker"

		self.detection_color = (0,0,255)
		self.device = self.get_parameter('device').get_parameter_value().string_value

		self.bridge = CvBridge()
		self.scan = None
		self.rgb_image_sub = self.create_subscription(Image, "/oak/rgb/image_raw", self.rgb_callback, qos_profile_sensor_data)
		self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

		self.marker_pub = self.create_publisher(Marker, marker_topic, QoSReliabilityPolicy.BEST_EFFORT)

		self.model = YOLO("yolov8n.pt")

		self.faces = []

		self.get_logger().info(f"Node has been initialized! Will publish face markers to {marker_topic}.")

	def rgb_callback(self, data):

		self.faces = []

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")


			# run inference
			res = self.model.predict(cv_image, imgsz=(256, 320), show=False, verbose=False, classes=[0], device=self.device)

			# iterate over results
			for x in res:
				bbox = x.boxes.xyxy
				if bbox.nelement() == 0: # skip if empty
					continue


				bbox = bbox[0]

				# draw rectangle
				cv_image = cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), self.detection_color, 3)

				cx = int((bbox[0]+bbox[2])/2)
				cy = int((bbox[1]+bbox[3])/2)

				# draw the center of bounding box
				cv_image = cv2.circle(cv_image, (cx,cy), 5, self.detection_color, -1)

				self.faces.append((cx,cy))
				print("list:", self.faces)

				marker = Marker()

				marker.header.frame_id = "/base_link"
				marker.header.stamp = data.header.stamp

				marker.type = 2
				marker.id = 0

				# Set the scale of the marker
				scale = 0.1
				marker.scale.x = scale
				marker.scale.y = scale
				marker.scale.z = scale

				# Set the color
				marker.color.r = 1.0
				marker.color.g = 1.0
				marker.color.b = 1.0
				marker.color.a = 1.0

				# Set the pose of the marker
				marker.pose.position.x = float((bbox[0]+bbox[2])/2)
				marker.pose.position.y = float((bbox[1]+bbox[3])/2)
				marker.pose.position.z = 1.

				#self.marker_pub.publish(marker)

			cv2.imshow("Face Recognition", cv_image)
			key = cv2.waitKey(1)
			if key==27:
				print("exiting")
				exit()
			
		except CvBridgeError as e:
			print(e)

	def pointcloud_callback(self, data):
		height = data.height
		width = data.width
		a = pc2.read_points_numpy(data, field_names=("x", "y", "z"))
		a = a.reshape((height, width, 3))

		# Assume camera (or sensor) position in the /base_link frame.
		# You might need to adjust this if your camera is offset.
		camera_position = np.array([0.0, 0.0, 0.0])
		
		for x, y in self.faces:
			window_size = 5  # Adjust window size if necessary
			points = []
			for j in range(max(0, y - window_size//2), min(height, y + window_size//2 + 1)):
				for i in range(max(0, x - window_size//2), min(width, x + window_size//2 + 1)):
					pt = a[j, i, :]
					if not np.isnan(pt[2]) and pt[2] != 0:
						points.append(pt)
			points = np.array(points)
			
			if points.shape[0] >= 3:
				centroid = np.mean(points, axis=0)
				pts_centered = points - centroid

				try:
					U, S, Vt = np.linalg.svd(pts_centered)
				except np.linalg.LinAlgError as e:
					self.get_logger().warn(f"SVD did not converge: {e}. Skipping this face.")
					continue  # Skip this face detection if SVD fails

				normal = Vt[-1, :]
				normal = normal / np.linalg.norm(normal)

				vec_to_camera = camera_position - centroid
				if np.dot(normal, vec_to_camera) < 0:
					normal = -normal

				
				offset_distance = 0.4  
				new_position = centroid + offset_distance * normal

				    
				vec_face = centroid - new_position
				
				yaw = np.arctan2(vec_face[1], vec_face[0])
				
				qz = np.sin(yaw / 2.0)
				qw = np.cos(yaw / 2.0)


				self.get_logger().info(
					f"Face normal: {normal}, Centroid: {centroid}, Offset position: {new_position}"
				)

				
				marker = Marker()
				marker.header.frame_id = "/base_link"
				marker.header.stamp = data.header.stamp
				marker.type = 2 
				marker.id = 0

				scale = 0.1
				marker.scale.x = scale
				marker.scale.y = scale
				marker.scale.z = scale

				marker.color.r = 1.0
				marker.color.g = 1.0
				marker.color.b = 1.0
				marker.color.a = 1.0

				marker.pose.orientation.x = 0.0
				marker.pose.orientation.y = 0.0
				marker.pose.orientation.z = qz
				marker.pose.orientation.w = qw

				marker.pose.position.x = float(new_position[0])
				marker.pose.position.y = float(new_position[1])
				marker.pose.position.z = float(new_position[2])
				self.marker_pub.publish(marker)


def main():
	print('Face detection node starting.')

	rclpy.init(args=None)
	node = detect_faces()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()