#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from undistorted import CameraReaderNode
from sensor_msgs.msg import CameraInfo

"""
Detects lane color, dimension of the tape and how far it is
DTROS not initialized to use the functions in lane based behaviour
"""

class LaneDetectionNode:

    def __init__(self, node_name):

        # Get vehicle name
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'default_duckiebot')
        self.focal_length_px = None  # Will be set in camera_info_callback


        # Define Topics
        #self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._lane_detection_topic = f"/{self._vehicle_name}/lane_detection/image/compressed"
        self.undistort = CameraReaderNode(node_name= "undistorted_image")
        self._camera_topic = f"/{self._vehicle_name}/camera_undistorted/image/compressed"
        self._camera_info_topic = f"/{self._vehicle_name}/camera_node/camera_info"

        # OpenCV Bridge
        self._bridge = CvBridge()

        # Subscribe to camera topic
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.image_callback, queue_size=1, buff_size=2**24)
        self.sub_info = rospy.Subscriber(self._camera_info_topic, CameraInfo, self.camera_info_callback)
        
        # Publisher for lane detection output
        self.pub = rospy.Publisher(self._lane_detection_topic, CompressedImage, queue_size=1)

        self.latest_image = None  # Store latest image for detection

        rospy.loginfo(f"LaneDetectionNode initialized. Subscribing to {self._camera_topic}")

        # Define HSV Color Ranges
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 100, 100])
        self.red_upper2 = np.array([180, 255, 255])

        self.yellow_lower = np.array([20, 100, 100])
        self.yellow_upper = np.array([40, 255, 255])
        

        self.blue_lower = np.array([100, 150, 100])  # Higher S (saturation) and V (brightness)
        self.blue_upper = np.array([130, 255, 255])

        self.green_lower = np.array([40, 100, 100])  # Make green detection stricter
        self.green_upper = np.array([85, 255, 255])

        self.brown_lower = np.array([10, 50, 20])   # Lower hue, low brightness
        self.brown_upper = np.array([30, 255, 180]) # Avoid overlap with yellow

    def image_callback(self, msg):
        """Store the latest image received from the camera."""
        try:
            self.latest_image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    
    def camera_info_callback(self, msg):
        """Receives camera intrinsic parameters and stores them."""
        self.camera_matrix = np.array(msg.K).reshape((3, 3))
        self.dist_coeffs = np.array(msg.D)
        self.focal_length_px = self.camera_matrix[0, 0]  # fx value from intrinsic matrix


    def detect_lanes(self):
        """Detect lanes using the latest stored image."""
        while self.latest_image is None and not rospy.is_shutdown():
            rospy.logwarn("Waiting for the first image...")
            rospy.sleep(0.7)  # Small delay to allow images to be received

        return self._process_lanes(self.latest_image)


    def _process_lanes(self, image):
        """Process image to detect lanes, compute dimensions, and estimate distance."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create masks
        red_mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        blue_mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        brown_mask = cv2.inRange(hsv, self.brown_lower, self.brown_upper)

        # Exclude yellow from red detection
        red_mask = cv2.bitwise_and(red_mask, cv2.bitwise_not(yellow_mask))

        # Exclude brown from all lane colors *only if brown is strongly detected*
        if cv2.countNonZero(brown_mask) > 500:
            red_mask = cv2.bitwise_and(red_mask, cv2.bitwise_not(brown_mask))
            blue_mask = cv2.bitwise_and(blue_mask, cv2.bitwise_not(brown_mask))
            green_mask = cv2.bitwise_and(green_mask, cv2.bitwise_not(brown_mask))

        # Apply morphological operations to remove small noise
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        # Count the number of detected pixels
        red_pixels = cv2.countNonZero(red_mask)
        blue_pixels = cv2.countNonZero(blue_mask)
        green_pixels = cv2.countNonZero(green_mask)

        # Determine dominant detected color
        detected_color = None
        detected_dimensions = None
        object_distance = None  # Store estimated distance

        # Find the largest detected object for each color and compute dimensions
        for mask, color, bgr in [(red_mask, "red", (0, 0, 255)), 
                                (blue_mask, "blue", (255, 0, 0)), 
                                (green_mask, "green", (0, 255, 0))]:
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)  # Get bounding box

                if w > 7 and h > 7:  # Ensure object is large enough to be relevant
                    detected_color = color
                    detected_dimensions = (w, h)  

                    # Compute object distance
                    object_distance = self.estimate_distance(w)  # Pass width in pixels

                    # Draw bounding box and label
                    cv2.rectangle(image, (x, y), (x + w, y + h), bgr, 3)
                    cv2.putText(image, f"{color.capitalize()} {round(object_distance, 2) if object_distance else 'Unknown'}m",
                                (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 2)
                    break  

        # Draw contours on the output image
        self.draw_contours(image, red_mask, (0, 0, 255))
        self.draw_contours(image, blue_mask, (255, 0, 0))
        self.draw_contours(image, green_mask, (0, 255, 0))

        return str(detected_color), detected_dimensions, object_distance, image


    def estimate_distance(self, object_pixel_width):
        """Estimates the distance to the object using the pinhole camera model."""
        if object_pixel_width == 0:
            rospy.logwarn("Object width in pixels is zero. Cannot estimate distance.")
            return None  

        if self.focal_length_px is None:
            rospy.logwarn("Focal length not received from camera_info. Using default.")
            self.focal_length_px = 500 
        known_object_width_m = 0.232  # 0.6666 ft converted to meters

        distance_m = (known_object_width_m * self.focal_length_px) / object_pixel_width
        return round(distance_m, 2)  # Round to two decimal places for readability


    def draw_contours(self, image, mask, color):
        """Finds contours in a binary mask and draws bounding rectangles around detected lanes."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 1000:  # Increased threshold to filter small noise
                x, y, w, h = cv2.boundingRect(contour)  # Get bounding box
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 3)  # Draw rectangle

                # Optionally, put text label
                label = "Lane" if color == (0, 255, 0) else "Red" if color == (0, 0, 255) else "Blue"
                cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

if __name__ == '__main__':
    node = LaneDetectionNode(node_name="lane_detection_node")
    rospy.spin()
