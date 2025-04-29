#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from ultralytics import YOLO
import tf2_ros
import tf2_geometry_msgs

class FaceDetector(Node):
	def __init__(self):
		super().__init__('detect_faces')

		self.declare_parameters(
			namespace='',
			parameters=[
				('device', ''),
		])
		self.device = self.get_parameter('device').get_parameter_value().string_value

		self.bridge = CvBridge()

		self.depth_image = None
		self.camera_info = None

		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

		self.create_subscription(
			Image,
			'/oak/rgb/image_raw',
			self.rgb_callback,
			qos_profile_sensor_data
		)
		self.create_subscription(
			Image,
			'/oak/stereo/image_raw',
			self.depth_callback,
			qos_profile_sensor_data
		)
		self.camera_info_subscriber = self.create_subscription(
			CameraInfo,
			'/oak/rgb/camera_info',
			self.camera_info_callback,
			qos_profile_sensor_data
		)

		self.marker_pub = self.create_publisher(
			Marker,
			'/people_marker',
			QoSReliabilityPolicy.BEST_EFFORT
		)

		self.model = YOLO('yolov8n.pt')
		self.get_logger().info('FaceDetector node initialized')

		cv2.namedWindow("Depth Image", cv2.WINDOW_NORMAL)
		cv2.moveWindow("Depth Image", 1, 1)
		cv2.namedWindow("Face Detection", cv2.WINDOW_NORMAL)
		cv2.moveWindow("Face Detection", 1, 351)

	def camera_info_callback(self, msg: CameraInfo):
		if self.camera_info is None:
			self.camera_info = msg
			self.get_logger().info("ℹ️ Received camera intrinsics, unsubscribing from topic!")
			self.destroy_subscription(self.camera_info_subscriber)

	def depth_callback(self, msg: Image):
		try:
			if msg.encoding == '16UC1':
				depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
				self.depth_image = depth_raw.astype(np.float32) / 1000.0
				disp_image = np.nan_to_num(self.depth_image)
				disp_image = np.clip(disp_image, 0, 5)
				disp_image = (disp_image / 5.0 * 255).astype(np.uint8)

				cv2.imshow("Depth Image", disp_image)
				cv2.waitKey(1)
			else:
				self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
		except CvBridgeError as e:
			self.get_logger().error(f'Depth convert error: {e}')
		

	def rgb_callback(self, msg: Image):
		if self.depth_image is None or self.camera_info is None:
			return

		try:
			color_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
		except CvBridgeError as e:
			self.get_logger().error(f'RGB conversion failed: {e}')
			return
		
		# Run face detection (class 0)
		results = self.model.predict(
			color_img,
			imgsz=(256, 320),
			classes=[0],
			device=self.device,
			verbose=False
		)

		# Extract camera intrinsics
		K = self.camera_info.k
		fx, fy = K[0], K[4]
		cx_i, cy_i = K[2], K[5]
		cam_frame = self.camera_info.header.frame_id

		for det in results:
			if det.boxes.xyxy.nelement() == 0:
				continue

			x1, y1, x2, y2 = det.boxes.xyxy[0]
			u = int((x1 + x2) / 2)
			v = int((y1 + y2) / 2)

			# Draw 2D overlay
			cv2.rectangle(color_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
			cv2.circle(color_img, (u, v), 4, (0, 0, 255), -1)

			# Sample a small patch around (u,v)
			patch = self.depth_image[
				max(0, v-1): v+2,
				max(0, u-1): u+2
			]
			valid = patch[np.isfinite(patch) & (patch > 0)]
			if valid.size == 0:
				self.get_logger().warn(f'No valid depth at pixel ({u},{v})')
				continue

			# Compute 3D point in camera frame
			Z = float(np.mean(valid))
			X = (u - cx_i) * Z / fx
			Y = (v - cy_i) * Z / fy

			# Build PoseStamped in camera frame
			cam_pose = PoseStamped()
			cam_pose.header.frame_id = cam_frame
			cam_pose.header.stamp = msg.header.stamp
			cam_pose.pose.position.x = X
			cam_pose.pose.position.y = Y
			cam_pose.pose.position.z = Z
			# orientation pointing back to camera
			yaw = np.arctan2(Y, X)
			cam_pose.pose.orientation.z = float(np.sin(yaw/2.0))
			cam_pose.pose.orientation.w = float(np.cos(yaw/2.0))


			# Transform into base_link

			t = self.tf_buffer.lookup_transform('base_link', cam_frame, msg.header.stamp, rclpy.duration.Duration(seconds=1.0))
			base_pose = tf2_geometry_msgs.do_transform_pose(cam_pose, t)



			self.get_logger().info("AAAAA")
			# Create and publish marker in base_link
			marker = Marker()

			marker.header.frame_id = "base_link"
			# marker.header.stamp = rospy.get_rostime()

			marker.type = 2
			marker.id = 0

			# Set the scale of the marker
			scale = 3.
			marker.scale.x = scale
			marker.scale.y = scale
			marker.scale.z = scale

			# Set the color
			marker.color.r = 1.0
			marker.color.g = 0.0
			marker.color.b = 0.0
			marker.color.a = 1.0
			marker.pose = base_pose.pose

			marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

			self.marker_publisher.publish(marker)

		# Show detection window
		cv2.imshow('Face Detection', color_img)
		if cv2.waitKey(1) == 27:
			rclpy.shutdown()


def main():
	rclpy.init()
	node = FaceDetector()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
