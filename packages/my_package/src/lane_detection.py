#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

"""
Detects lane color
DTROS not initialized to use the functions in lane based behaviour
"""

class LaneDetectionNode:

    def __init__(self, node_name):

        # Get vehicle name
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'default_duckiebot')

        # Define Topics
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._lane_detection_topic = f"/{self._vehicle_name}/lane_detection/image/compressed"

        # OpenCV Bridge
        self._bridge = CvBridge()

        # Subscribe to camera topic
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.image_callback, queue_size=1, buff_size=2**24)
        
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

        self.green_lower = np.array([46, 50, 65])
        self.green_upper = np.array([95, 196, 199])

        self.brown_lower = np.array([10, 50, 20])   # Lower hue, low brightness
        self.brown_upper = np.array([30, 255, 180]) # Avoid overlap with yellow

    def image_callback(self, msg):
        """Store the latest image received from the camera."""
        try:
            self.latest_image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def detect_lanes(self):
        """Detect lanes using the latest stored image."""
        while self.latest_image is None and not rospy.is_shutdown():
            rospy.logwarn("Waiting for the first image...")
            rospy.sleep(0.7)  # Small delay to allow images to be received

        return self._process_lanes(self.latest_image)


        return self._process_lanes(self.latest_image)

    def _process_lanes(self, image):
        """Process image to detect lanes and return detected color."""
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

        # Exclude brown from all lane colors
        red_mask = cv2.bitwise_and(red_mask, cv2.bitwise_not(brown_mask))
        blue_mask = cv2.bitwise_and(blue_mask, cv2.bitwise_not(brown_mask))
        green_mask = cv2.bitwise_and(green_mask, cv2.bitwise_not(brown_mask))

        # Apply morphological operations to remove small noise
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        # Get the number of detected pixels for each color
        red_pixels = cv2.countNonZero(red_mask)
        blue_pixels = cv2.countNonZero(blue_mask)
        green_pixels = cv2.countNonZero(green_mask)

        # Determine the dominant detected color
        detected_color = None
        if red_pixels > 1000:  # Increased threshold to remove tiny detections
            detected_color = "red"
        elif blue_pixels > 1000:
            detected_color = "blue"
        elif green_pixels > 1000:
            detected_color = "green"

        # Draw contours around detected lanes
        output = image.copy()
        self.draw_contours(output, red_mask, (0, 0, 255))  # Red contours
        self.draw_contours(output, blue_mask, (255, 0, 0))  # Blue contours
        self.draw_contours(output, green_mask, (0, 255, 0))  # Green contours
        return str(detected_color), output  # Convert to Python string

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
