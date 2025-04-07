#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from visualization_msgs.msg import Marker

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from ultralytics import YOLO

import tf2_ros

from geometry_msgs.msg import PointStamped, Vector3, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from sensor_msgs.msg import CameraInfo

qos_profile = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

class detect_rings(Node):
    def __init__(self):
        super().__init__('detect_rings')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('device', ''),
        ])
        
        timer_frequency = 2
        timer_period = 1/timer_frequency

        marker_topic = "/ring_marker"
        self.detection_color = (0,0,255)
        self.device = self.get_parameter('device').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.scan = None

        self.image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.image_callback, 1)
        self.depth_sub = self.create_subscription(Image, "/oakd/rgb/preview/depth", self.depth_callback, 1)
        self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

        self.marker_pub = self.create_publisher(Marker, marker_topic, QoSReliabilityPolicy.BEST_EFFORT)
        
        self.marker_array = MarkerArray()
        self.marker_num = 1
        
        #self.tf_buf = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.model = YOLO("yolov8n.pt")

        self.rings = []
        self.rings_global = []

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_received = False
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/oakd/rgb/preview/camera_info",  # <- Adjust if you're using a different topic
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(f"Node has been initialized! Will publish face markers to {marker_topic}.")
        
        #cv2.namedWindow("Binary Image", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Detected contours", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Detected rings", cv2.WINDOW_NORMAL)
        #cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)   
        cv2.namedWindow("Ring color", cv2.WINDOW_NORMAL)   

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_info_received = True

    def image_callback(self, data):
        #self.get_logger().info(f"I got a new image! Will try to find rings...")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        blue = cv_image[:,:,0]
        green = cv_image[:,:,1]
        red = cv_image[:,:,2]

        ## for red rings

        # set color range for HSV red
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        # masks for red
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask1, mask2)

        # connect gaps
        kernel = np.ones((5, 5), np.uint8)

        # phological closing
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

        #cv2.imshow("Red Mask", mask_red)
        #cv2.waitKey(1)

        thresh_red = mask_red

        ## for 3D rings (dark green)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # dark green hsv
        lower_green = np.array([45, 30, 15])
        upper_green = np.array([90, 255, 150])

        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # connect gaps
        kernel = np.ones((5, 5), np.uint8)

        # morphological closing
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)


        darkgreen_thresh = mask_green

        #cv2.imshow("Dark Green Mask", mask_green)
        #cv2.waitKey(1)


        ## for other rings (green)

        # Tranform image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # gray = red

        # Apply Gaussian Blur
        #gray = cv2.GaussianBlur(gray,(3,3),0)
        
        # Do histogram equalization
        #gray = cv2.equalizeHist(gray)

        # Binarize the image, there are different ways to do it
        #ret, thresh = cv2.threshold(img, 50, 255, 0)
        #ret, thresh = cv2.threshold(img, 70, 255, cv2.THRESH_BINARY)

        thresh_main = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 30)

        #cv2.imshow("Binary Image", thresh_main)
        #cv2.waitKey(1)
        # Extract contours
        contours, hierarchy = cv2.findContours(thresh_main, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Example of how to draw the contours, only for visualization purposes
        cv2.drawContours(gray, contours, -1, (255, 0, 0), 3)
        cv2.imshow("Detected contours", gray)
        cv2.waitKey(1)

        # Fit elipses to all extracted contours
        elps = []
        for cnt in contours:
            #    print cnt
            #    print cnt.shape
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)


        contours_red, hierarchy = cv2.findContours(thresh_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_red:
            #    print cnt
            #    print cnt.shape
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)

        contours_darkgreen, hierarchy = cv2.findContours(darkgreen_thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_darkgreen:
            #    print cnt
            #    print cnt.shape
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)


        # Find two elipses with same centers
        candidates = []
        for n in range(len(elps)):
            for m in range(n + 1, len(elps)):
                # e[0] is the center of the ellipse (x,y), e[1] are the lengths of major and minor axis (major, minor), e[2] is the rotation in degrees
                
                e1 = elps[n]
                e2 = elps[m]
                dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
                angle_diff = np.abs(e1[2] - e2[2])

                # The centers of the two elipses should be within 5 pixels of each other (is there a better treshold?)
                if dist >= 5:
                    continue

                # The rotation of the elipses should be whitin 4 degrees of eachother
                if angle_diff>10:
                    continue

                e1_minor_axis = e1[1][0]
                e1_major_axis = e1[1][1]

                e2_minor_axis = e2[1][0]
                e2_major_axis = e2[1][1]

                if e1_major_axis>=e2_major_axis and e1_minor_axis>=e2_minor_axis: # the larger ellipse should have both axis larger
                    le = e1 # e1 is larger ellipse
                    se = e2 # e2 is smaller ellipse
                elif e2_major_axis>=e1_major_axis and e2_minor_axis>=e1_minor_axis:
                    le = e2 # e2 is larger ellipse
                    se = e1 # e1 is smaller ellipse
                else:
                    continue # if one ellipse does not contain the other, it is not a ring
                
                # # The widths of the ring along the major and minor axis should be roughly the same
                border_major = (le[1][1]-se[1][1])/2
                border_minor = (le[1][0]-se[1][0])/2
                border_diff = np.abs(border_major - border_minor)

                # should be circular enough
                if(e1_major_axis > e1_minor_axis * 2):
                    continue
                if(e2_major_axis > e2_minor_axis * 2):
                    continue

                # should be big enough (so no QR code)
                if(e1_major_axis < 20 or e2_major_axis < 20):
                    continue

                #print("Ellipse", e1_minor_axis, e1_major_axis, e2_minor_axis, e2_major_axis, dist, angle_diff, border_diff, border_minor, border_major)

                if border_diff>2:
                    continue
                
                candidates.append((e1,e2))
                
        if(len(candidates) != 0):
            #print("Processing is done! found", len(candidates), "candidates for rings.")
            pass
            

        # Plot the rings on the image
        for c in candidates:

            # the centers of the ellipses
            e1 = c[0]
            e2 = c[1]

            # Safe image for visualization
            viz_image = cv_image.copy()

            # Draw on the viz image, NOT the one used for color
            cv2.ellipse(viz_image, e1, (0, 255, 0), 2)
            cv2.ellipse(viz_image, e2, (0, 255, 0), 2)


            ### COLOR DETECTION
            # Determine which ellipse is larger
            def ellipse_area(e):
                return e[1][0] * e[1][1]

            if ellipse_area(e1) < ellipse_area(e2):
                e1, e2 = e2, e1  # Swap so e1 is larger

            # Create mask
            mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)

            # Draw outer (larger) ellipse
            cv2.ellipse(mask, e1, 255, -1)

            # Draw inner (smaller) ellipse to subtract
            cv2.ellipse(mask, e2, 0, -1)

            # Debug: check if mask is valid
            masked_pixels = cv_image[mask == 255]
            if len(masked_pixels) == 0:
                #self.get_logger().warn("No valid pixels found in mask — skipping color estimation.")
                continue



            avg_color = np.mean(masked_pixels, axis=0)
            b, g, r = avg_color.astype(int)

            #2 get average color
            pixels = cv_image[mask == 255]
            if len(pixels) == 0:
                continue

            avg_color = np.mean(pixels, axis=0)
            b, g, r = avg_color.astype(int)
            #print("Found candidate:", r, g, b)
            ring = {
                "pos": (int(e1[0][0]), int(e1[0][1])),
                "color": (r, g, b)
            }
            self.rings.append(ring)


            #DEBUG RING COLO
            text = str((r, g, b))
            coordinates = (10,100)
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2

            ring_only = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            ring_only = cv2.putText(ring_only, text, coordinates, font, fontScale, color, thickness, cv2.LINE_AA)

            cv2.imshow("Ring color", ring_only)
            cv2.waitKey(1)


            # Get a bounding box, around the first ellipse ('average' of both elipsis)
            size = (e1[1][0]+e1[1][1])/2
            center = (e1[0][1], e1[0][0])

            #self.get_logger().info(f"Ring at ({center[0]}, {center[1]}) with color (R: {r}, G: {g}, B: {b}), Rings count: {len(self.rings)}\nList: {self.rings}")

            x1 = int(center[0] - size / 2)
            x2 = int(center[0] + size / 2)
            x_min = x1 if x1>0 else 0
            x_max = x2 if x2<cv_image.shape[0] else cv_image.shape[0]

            y1 = int(center[1] - size / 2)
            y2 = int(center[1] + size / 2)
            y_min = y1 if y1 > 0 else 0
            y_max = y2 if y2 < cv_image.shape[1] else cv_image.shape[1]

        if len(candidates)>0:
                cv2.imshow("Detected rings",viz_image)
                cv2.waitKey(1)

    
    def add_ring(self, input_ring):
        new_ring = input_ring["pos"]
        # Define a distance threshold for what counts as "too close"
        distance_threshold = 2
        
        # Check if the new ring is too close to any existing ring
        for ring_dict in self.rings:
            existing_ring = ring_dict["pos"]
            distance = self.calculate_distance(new_ring, existing_ring)
            if distance < distance_threshold:
                #print("Ring is too close to an existing ring, skipping addition.")
                return  # Skip adding this ring
        
        # If the ring is far enough, add it to the list
        input_ring["pos"] = (int(new_ring[0]), int(new_ring[1]))
        self.rings.append(input_ring)
        #print(f"Added new ring at {input_ring}")

    def calculate_distance(self, ring1, ring2):
        # Calculate Euclidean distance between two rings (given as [x, y, z] coordinates)
        x1, y1 = ring1
        x2, y2 = ring2
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def depth_callback(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)
            return

        # Set invalid depth (inf) values to 0
        depth_image[np.isinf(depth_image)] = 0

        # Set the minimum depth threshold to filter out sky or invalid areas
        min_depth = 0.1  # Depth threshold to ignore very close points
        max_depth = 10.0  # Max depth to avoid points too far away (optional)

        # Assuming you already have these from a CameraInfo message
        fx = self.fx  # Focal length in x (in pixels)
        fy = self.fy  # Focal length in y (in pixels)
        cx = self.cx  # Optical center x (in pixels)
        cy = self.cy  # Optical center y (in pixels)

        for ring in self.rings:
            cx_ring, cy_ring = ring["pos"]  # Center pixel
            color = ring.get("color", (0, 0, 0))  # RGB tuple or fallback

            cx_ring, cy_ring = int(cx_ring), int(cy_ring)

            # Initialize a list to store the 3D coordinates for the valid ring pixels
            ring_points_3d = []

            # Look around the ring for valid depth (larger window)
            window_size = 10  # Increase the window size
            for dx in range(-window_size, window_size + 1):  # Scan over a larger window
                for dy in range(-window_size, window_size + 1):
                    ex, ey = cx_ring + dx, cy_ring + dy

                    # Make sure we're within the bounds of the image
                    if 0 <= ex < depth_image.shape[1] and 0 <= ey < depth_image.shape[0]:
                        d = depth_image[ey, ex]
                        
                        # Debugging: print the depth for surrounding pixels
                        #print(f"Depth at ({ex}, {ey}): {d}")

                        # Ensure depth is valid and within a reasonable range
                        if np.isfinite(d) and min_depth < d < max_depth:
                            # Calculate the 3D position for this pixel
                            z = d
                            x = (ex - cx) * z / fx  # Convert to real-world X
                            y = (ey - cy) * z / fy  # Convert to real-world Y

                            # Append this 3D point to the list
                            ring_points_3d.append([x, y, z])

            # If we have valid 3D points, compute the average position of the ring
            if ring_points_3d:
                position = np.mean(ring_points_3d, axis=0)
                r, g, b = ring["color"]  # RGB values for the ring

                # Publish a marker for visualization
                marker = Marker()
                marker.header.frame_id = "camera_link"  # or whatever frame your depth image is in
                marker.header.stamp = data.header.stamp
                marker.type = Marker.SPHERE
                marker.id = 0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.1

                marker.color.r = float(r)
                marker.color.g = float(g)
                marker.color.b = float(b)
                marker.color.a = 1.0

                # Set the marker pose based on the calculated position
                marker.pose.position.x = float(position[0])
                marker.pose.position.y = float(position[1])
                marker.pose.position.z = float(position[2])

                self.marker_pub.publish(marker)

                #self.rings_global.append({
                #    "pos": position,
                #    "color": ring["color"]
                #})

                #print("added new ring in depth: ", position, ring["color"])

            else:
                print(f"⛔ No valid depth values for ring at ({cx_ring}, {cy_ring}) | Color: R={r} G={g} B={b}")

        # Optional: clear ring detections
        self.rings = []







    def scale_color(color_16bit):
        return tuple(int(c / 256) for c in color_16bit)  # 65536 / 256 = 256


    def pointcloud_callback(self, data):
        return
        height = data.height
        width = data.width
        a = pc2.read_points_numpy(data, field_names=("x", "y", "z"))
        a = a.reshape((height, width, 3))

        # Assume camera (or sensor) position in the /base_link frame.
        # You might need to adjust this if your camera is offset.
        camera_position = np.array([0.0, 0.0, 0.0])
        
        for ring in self.rings:
            x, y = ring["pos"]
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
                if((str(pts_centered[0][0]) == "nan") or (str(pts_centered[0][0]) == "-inf") or (str(pts_centered[0][0]) == "inf")):
                    print("weird infinite val")
                    print(points, "\n", centroid, "\n", pts_centered)
                    continue
                U, S, Vt = np.linalg.svd(pts_centered)
                normal = Vt[-1, :]
                normal = normal / np.linalg.norm(normal)

                vec_to_camera = camera_position - centroid
                if np.dot(normal, vec_to_camera) < 0:
                    normal = -normal

                
                offset_distance = 0.4  
                new_position = centroid + offset_distance * normal

                #self.get_logger().info(
                #    f"Ring normal: {normal}, Centroid: {centroid}, Offset position: {new_position}, Color: {ring[1]}, List: {self.rings_global}"
                #)

                
                marker = Marker()
                marker.header.frame_id = "/base_link"
                marker.header.stamp = data.header.stamp
                marker.type = 2 
                marker.id = 0

                scale = 0.1
                marker.scale.x = scale
                marker.scale.y = scale
                marker.scale.z = scale

                r, g, b = ring["color"]
                marker.color.r = float(r)
                marker.color.g = float(g)
                marker.color.b = float(b)
                marker.color.a = 1.0

                marker.pose.position.x = float(new_position[0])
                marker.pose.position.y = float(new_position[1])
                marker.pose.position.z = float(new_position[2])
                new_ring = {
                    "pos": new_position,
                    "color": (float(r), float(g), float(b))
                }
                #self.add_global_ring(new_ring)
                self.rings_global.append(new_ring)
                self.marker_pub.publish(marker)
                self.rings = []
                self.rings_global = []

    def add_global_ring(self, new_ring):
        # Define a distance threshold for what counts as "too close"
        distance_threshold = 0.3
        
        # Check if the new ring is too close to any existing ring
        for ring in self.rings_global:
            existing_ring = ring["pos"]
            distance = self.calculate_3d_distance(new_ring["pos"], existing_ring)
            if distance < distance_threshold:
                #print("Ring is too close to an existing ring, skipping addition.")
                return  # Skip adding this ring
        
        self.rings_global.append(new_ring)
        print(f"Added new global ring at {new_ring['pos'], new_ring['color']}, List: {self.rings_global}")

    def calculate_3d_distance(self, ring1, ring2):
        # Calculate Euclidean distance between two rings (given as [x, y, z] coordinates)
        x1, y1, z1 = ring1
        x2, y2, z2 = ring2
        
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

def main():
	print('Ring detection node starting.')

	rclpy.init(args=None)
	node = detect_rings()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()