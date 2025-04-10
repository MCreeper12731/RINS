#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Image

from visualization_msgs.msg import Marker

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from ultralytics import YOLO

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

        
        cv2.namedWindow("Detected contours", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Detected rings", cv2.WINDOW_NORMAL)

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_info_received = True

    def is_circular(ellipse, tolerance=0.2):
        major = ellipse[1][1]
        minor = ellipse[1][0]
        ratio = minor / major if major != 0 else 0
        return ratio > (1 - tolerance)


    def image_callback(self, data):
        #self.get_logger().info(f"I got a new image! Will try to find rings...")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        masks = []
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Red color ranges
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        # Green color ranges
        lower_green = np.array([45, 30, 15])
        upper_green = np.array([90, 255, 150])

        # Blue color ranges
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([140, 255, 200])

        # Yellow color ranges
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])

        # Orange color ranges
        lower_orange = np.array([5, 150, 150])
        upper_orange = np.array([25, 255, 255])

        # Black color ranges
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])

        # White color ranges
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])



        # Red ring detection
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask1, mask2)
        thresh_red = cv2.adaptiveThreshold(mask_red, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 5, 20)
        masks.append([thresh_red, "RED"])

        # Green ring detection
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        thresh_green = cv2.adaptiveThreshold(mask_green, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 5, 20)
        masks.append([thresh_green, "GREEN"])

        # Blue ring detection
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        thresh_blue = cv2.adaptiveThreshold(mask_blue, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 5, 20)
        masks.append([thresh_blue, "BLUE"])

        # Yellow ring detection
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        thresh_yellow = cv2.adaptiveThreshold(mask_yellow, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 5, 20)
        masks.append([thresh_yellow, "YELLOW"])

        # Orange ring detection
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
        thresh_orange = cv2.adaptiveThreshold(mask_orange, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 5, 20)
        masks.append([thresh_orange, "ORANGE"])

        # Black ring detection
        mask_black = cv2.inRange(hsv, lower_black, upper_black)
        thresh_black = cv2.adaptiveThreshold(mask_black, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 5, 20)
        masks.append([thresh_black, "BLACK"])

        # White ring detection
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        thresh_white = cv2.adaptiveThreshold(mask_white, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 5, 20)
        masks.append([thresh_white, "WHITE"])


        ## for other rings

        # Tranform image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)


        # connect gaps
        for i in range(len(masks)):
            kernel = np.ones((3, 3), np.uint8)
            masks[i][0] = cv2.morphologyEx(masks[i][0], cv2.MORPH_CLOSE, kernel)

        thresh_main = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 15, 30)

        kernel = np.ones((5, 5), np.uint8)
        thresh_main = cv2.morphologyEx(thresh_main, cv2.MORPH_CLOSE, kernel)

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
                elps.append((ellipse, "MAIN"))


        for mask, color in masks:
            contours_specific, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours_specific:
                #    print cnt
                #    print cnt.shape
                if cnt.shape[0] >= 20:
                    ellipse = cv2.fitEllipse(cnt)
                    elps.append((ellipse, color))


        # Find two elipses with same centers
        candidates = []
        for n in range(len(elps)):
            for m in range(n + 1, len(elps)):
                # e[0] is the center of the ellipse (x,y), e[1] are the lengths of major and minor axis (major, minor), e[2] is the rotation in degrees

                e1 = elps[n][0]
                e2 = elps[m][0]
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
                #print(e1_major_axis, e1_minor_axis * 2, e2_major_axis, e2_minor_axis * 2)
                if(e1_major_axis > e1_minor_axis * 2):
                    continue
                if(e2_major_axis > e2_minor_axis * 2):
                    continue
                
                # rings should be similiar size, not too different
                if(e1_minor_axis > e2_minor_axis * 1.6):
                    return
                if(e1_major_axis > e2_major_axis * 1.6):
                    return

                # should be big enough (so no QR code)
                if(e1_major_axis < 12 or e2_major_axis < 12):
                    #print(e1_major_axis, e2_major_axis)
                    continue

                #print("Ellipse", e1_minor_axis, e1_major_axis, e2_minor_axis, e2_major_axis, dist, angle_diff, border_diff, border_minor, border_major)

                if border_diff>2:
                    continue
                
                candidates.append(((e1,e2), elps[m][1]))
                
        if(len(candidates) != 0):
            #print("Processing is done! found", len(candidates), "candidates for rings.")
            pass
            

        # Plot the rings on the image
        for c in candidates:

            # the centers of the ellipses
            e1 = c[0][0]
            e2 = c[0][1]

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
                "color": (r, g, b),
                "viz_image": viz_image,
                "image": cv_image,
                "mask": mask,
                "original_color": c[1]
            }
            self.rings.append(ring)


            #DEBUG RING COLOR
            text = str((r, g, b))
            coordinates = (10,100)
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2

            ring_only = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            ring_only = cv2.putText(ring_only, text, coordinates, font, fontScale, color, thickness, cv2.LINE_AA)

            #cv2.imshow("Ring color", ring_only)
            #cv2.waitKey(1)

            ### CHECKING FOR SPHERE
            # Check the center pixel inside the smaller ellipse
            center_x, center_y = int(e2[0][0]), int(e2[0][1])  # smaller inner ellipse center

            # Ensure center pixel is in image bounds
            if 0 <= center_y < cv_image.shape[0] and 0 <= center_x < cv_image.shape[1]:
                center_pixel_color = cv_image[center_y, center_x].astype(int)
                center_r, center_g, center_b = center_pixel_color

                # Compute Euclidean distance in RGB color space
                color_distance = np.linalg.norm(np.array([r, g, b]) - np.array([center_r, center_g, center_b]))

                # If color is too similar, it's probably a sphere, not a ring
                if color_distance < 30:  # adjust threshold if needed
                    continue  # skip this candidate



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


    def depth_callback(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)
            return
        

        # Set invalid depth (inf) values to 0
        old_depth_image = depth_image.copy()
        depth_image[np.isinf(depth_image)] = 0

        # Set the minimum depth threshold to filter out sky or invalid areas
        min_depth = 0.1  # Depth threshold to ignore very close points
        max_depth = 10.0  # Max depth to avoid points too far away (optional)

        # Assuming you already have these from a CameraInfo message
        fx = self.fx  # Focal length in x (in pixels)
        fy = self.fy  # Focal length in y (in pixels)
        cx = self.cx  # Optical center x (in pixels)
        cy = self.cy  # Optical center y (in pixels)

        if None in (self.fx, self.fy, self.cx, self.cy):
            #print("Camera intrinsics not yet received, skipping depth callback")
            return

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
            # Check if we have valid 3D points on the ring

            if ring_points_3d:
                # Get center depth at the ellipse center
                if 0 <= cx_ring < depth_image.shape[1] and 0 <= cy_ring < depth_image.shape[0]:
                    depth_center = depth_image[cy_ring, cx_ring]
                else:
                    continue  # Skip if out of bounds

                # Clean and analyze ring depth
                ring_zs = [pt[2] for pt in ring_points_3d if np.isfinite(pt[2])]
                if not ring_zs:
                    continue  # No valid depths on ring

                depth_ring_median = np.median(ring_zs)

                #print(depth_center, depth_ring_median)

                # If center is invalid or farther away than the ring, it's probably a ring not a sphere
                if(depth_center * 0.9 > depth_ring_median or depth_center == 0):
                    print(f" FOUND RING({cx_ring}, {cy_ring}) — depth_center={depth_center}, ring_median={depth_ring_median}, old: {old_depth_image[cy_ring, cx_ring]}, color: {ring['color']}, likely color: {self.classify_color(ring['color'])}, original color: {ring['original_color']}")
                    pass
                else:
                    #print(f" Skipping sphere at ({cx_ring}, {cy_ring}) — depth_center={depth_center} >= ring_median={depth_ring_median}, depth * 0.9: {depth_center * 0.9}")
                    continue

                if(ring['original_color'] != self.classify_color(ring['color'])):
                    continue

                # Passed the check — this is likely a ring
                position = np.mean(ring_points_3d, axis=0)
                r, g, b = ring["color"]  # RGB values for the ring

                marker = Marker()
                marker.header.frame_id = "camera_link"
                marker.header.stamp = data.header.stamp
                marker.type = Marker.SPHERE
                marker.id = 0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.1

                marker.color.r = float(r)
                marker.color.g = float(g)
                marker.color.b = float(b)
                marker.color.a = 1.0

                marker.pose.position.x = float(position[0])
                marker.pose.position.y = float(position[1])
                marker.pose.position.z = float(position[2])

                self.marker_pub.publish(marker)

                cv2.imshow("Detected rings",ring["viz_image"])
                cv2.waitKey(1)

                #ring_only = cv2.bitwise_and(ring["image"], ring["image"], mask=ring["mask"])
                #cv2.imshow("Ring color", ring_only)
                #cv2.waitKey(1)


        # Optional: clear ring detections
        self.rings = []

        ##DEBUG WINDOW
        # Do the necessairy conversion so we can visuzalize it in OpenCV
        image_1 = depth_image / 65536.0 * 255
        image_1 = image_1/np.max(image_1)*255

        image_viz = np.array(image_1, dtype= np.uint8)

    def classify_color(self, rgb):
        r = rgb[0]
        g = rgb[1]
        b = rgb[2]

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



def main():
	print('Ring detection node starting.')

	rclpy.init(args=None)
	node = detect_rings()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()