#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage


import cv2
from cv_bridge import CvBridge
import numpy as np


class ControllerNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ControllerNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._yellow_line_detection = f"/{self._vehicle_name}/camera_processed_yellow/image/compressed"
        self._white_line_detection = f"/{self._vehicle_name}/camera_processed_white/image/compressed"

        # bridge between OpenCV and ROS
        self._bridge = CvBridge()

        # Subscribe to the camera topic
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        # Publisher for lane detection results
        self.pub_yellow = rospy.Publisher(self._yellow_line_detection, CompressedImage, queue_size=10)
        self.pub_white = rospy.Publisher(self._white_line_detection, CompressedImage, queue_size=10)

    def callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

            height, width, _ = cv_image.shape
            print(f"Received image size: {width}x{height}")

            # Crop the bottom half of the image
            bottom_half = cv_image[height // 4:, :]  # Select rows from middle to the end

            # Detect yellow dotted lane
            yellow_lanes = self.detect_yellow_dotted_lane(bottom_half)

            # detect white solid line
            white_lanes = self.detect_white_solid_lane(bottom_half)

            processed_msg_yellow = self._bridge.cv2_to_compressed_imgmsg(yellow_lanes)
            processed_msg_white = self._bridge.cv2_to_compressed_imgmsg(white_lanes)

            self.pub_yellow.publish(processed_msg_yellow)
            self.pub_white.publish(processed_msg_white)

        

        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

    
    def detect_yellow_dotted_lane(self, image):
        # Convert the image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the range for the yellow color in HSV space
        lower_yellow = np.array([20, 75, 100])
        upper_yellow = np.array([30, 255, 255])

        # Mask the image to get only the yellow colors
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        return yellow_mask
    
    def detect_white_solid_lane(self, image):
        # Convert the image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the range for the white color in HSV space
        lower_white = np.array([0, 0, 165])     # Allow all hues, low saturation, high brightness
        upper_white = np.array([180, 29, 255])   # High value, low saturation

        # Mask the image to get only the white colors
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        return white_mask



if __name__ == '__main__':
    # create the node
    node = ControllerNode(node_name='controller_node')
    # keep spinning
    rospy.spin()
