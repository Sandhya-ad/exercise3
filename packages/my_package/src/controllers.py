#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage, Image


import cv2
from cv_bridge import CvBridge
import numpy as np

# Throttle and direction for each wheel
THROTTLE_LEFT = 0.23  # 50% throttle
DIRECTION_LEFT = 1  # Forward
THROTTLE_RIGHT = 0.2  # 50% throttle
DIRECTION_RIGHT = 1  # Forward



class ControllerNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ControllerNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # topics for duckie vision to detect lane 
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._yellow_line_detection = f"/{self._vehicle_name}/camera_processed_yellow/image"
        self._white_line_detection = f"/{self._vehicle_name}/camera_processed_white/image"
        self._both = f"/{self._vehicle_name}/camera_processed_both/image/compressed"

        # topics related to wheels
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        self._wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"

        # Form the message to run the Duckiebot
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT


        # Temporary data storage for measure distance
        self._ticks_left = None
        self._ticks_right = None
        self._initial_ticks_left = None
        self._initial_ticks_right = None
         

        # camerta intrinsic parameters
        self.camera_matrix = np.array([[263.6565, 0.0, 333.3401],
                                       [0.0, 265.4119, 210.1412],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.array([-0.2147, 0.03395, 0.008495, 0.0004646, 0.0])


        # bridge between OpenCV and ROS
        self._bridge = CvBridge()

        # Subscribe to the camera topic
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        # Publisher for lane detection (colors)
        self.pub_yellow = rospy.Publisher(self._yellow_line_detection, Image, queue_size=1)
        self.pub_white = rospy.Publisher(self._white_line_detection, Image, queue_size=1)
        self.pub_both = rospy.Publisher(self._both, CompressedImage, queue_size=1)
        
        # Construct subscribers and publishers for wheels
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        self.publisher = rospy.Publisher(self._wheels_topic, WheelsCmdStamped, queue_size=1)


        # variables for Controller
        self.Kp = 0.007   # Tune this value absed on testing ask chatGPT more about it how increase 
        # and decrease affect the movement

         # Variables for image processing optimization
        self.last_processed_image = None  # Stores the last cropped image
        self.difference_threshold = 5    # Adjust threshold as necessary
        

    def callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

            height, width, _ = cv_image.shape
            #print(f"Received image size: {width}x{height}")

            # Crop the bottom half of the image
            bottom_half = cv_image[height // 2:, :]  # Select rows from middle to the end

            # Check if the current cropped image is significantly different from the last processed image
            if self.last_processed_image is not None:
                diff = cv2.absdiff(bottom_half, self.last_processed_image)
                mean_diff = np.mean(diff)
                # If the difference is very low, skip further processing
                if mean_diff < self.difference_threshold:
                    return

            # Update the last processed image (copy to avoid reference issues)
            self.last_processed_image = bottom_half.copy()


            # undistort the image
            undistorted_image = self.undistort_image(bottom_half)

            # Detect yellow dotted lane
            yellow_lanes_masking = self.detect_yellow_dotted_lane(undistorted_image)

            # detect white solid line
            white_lane_masking = self.detect_white_solid_lane(undistorted_image)

            # combine both lane masking
            Combined_mask = cv2.bitwise_or(yellow_lanes_masking, white_lane_masking)

            processed_msg_yellow = self._bridge.cv2_to_imgmsg(yellow_lanes_masking)
            processed_msg_white = self._bridge.cv2_to_imgmsg(white_lane_masking)
            processed_msg_both = self._bridge.cv2_to_compressed_imgmsg(undistorted_image)

            # Calculate the lane center
            lane_center = self.compute_lane_center(yellow_lanes_masking, white_lane_masking, undistorted_image)

            # Apply P controller to adjust steering
            if lane_center != None:
                self.apply_p_controller(lane_center, undistorted_image)


            self.pub_yellow.publish(processed_msg_yellow)
            self.pub_white.publish(processed_msg_white)
            self.pub_both.publish(processed_msg_both)


        

        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

        
    def callback_left(self, data):
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        if self._initial_ticks_left is None:
            self._initial_ticks_left = data.data
        self._ticks_left = data.data

    def callback_right(self, data):
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")
        if self._initial_ticks_right is None:
            self._initial_ticks_right = data.data
        self._ticks_right = data.data


    def undistort_image(self, image):
        h, w = image.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
        # self.map1, self.map2 = cv2.initUndistortRectifyMap(
         #    self.camera_matrix, self.dist_coeffs, None, self.new_camera_matrix, (w, h), cv2.CV_16SC2)
        
        # undistorted = cv2.remap(image, self.map1, self.map2, cv2.INTER_LINEAR)
        # image = cv2.resize(image, (320, 240))  # Adjust resolution as needed
        # return cv2.GaussianBlur(image, (5, 5), 0)
        undistorted = cv2.undistort(image, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)

        
        undistorted = cv2.resize(undistorted, (320, 240))  # Adjust resolution as needed
        #return cv2.GaussianBlur(undistorted, (5, 5), 0)
        return undistorted

    
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
        lower_white = np.array([0, 0, 135])     # Allow all hues, low saturation, high brightness
        upper_white = np.array([180, 29, 255])   # High value, low saturation

        # Mask the image to get only the white colors
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        return white_mask
        
    def compute_lane_center(self, yellow_mask, white_mask, image):
        # Find contours
        contours_white, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_yellow, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Get centroids for both lanes (handling missing contours)
        centroid_white = self.find_centroid(contours_white) if len(contours_white) > 0 else None
        centroid_yellow = self.find_centroid(contours_yellow) if len(contours_yellow) > 0 else None

        # Assuming the robot is in the center of the image
        image_center = image.shape[1] // 2  # Horizontal center of the image



        # Handle cases where one or both centroids are missing
        if centroid_white is None and centroid_yellow is None:
            #print("No lanes detected.")
            return None  # Default to image center to prevent extreme corrections

        if centroid_white is None:
            lane_center = centroid_yellow[0]  # Use only yellow lane
        elif centroid_yellow is None:
            lane_center = centroid_white[0]  # Use only white lane
        else:
            lane_center = (centroid_yellow[0] + centroid_white[0]) // 2  # Average both lanes

        return lane_center  # Return the x-coordinate of the lane center




        '''
        # for Testing
        # to check which line is closer

        # Handle cases where one or both centroids are missing
        if centroid_white is None and centroid_yellow is None:
            print("No lanes detected.")
            return None

        if centroid_white is None:
            return "yellow"

        if centroid_yellow is None:
            return "white"

        # Calculate distance to both lanes
        distance_to_white = abs(image_center - centroid_white[0])
        distance_to_yellow = abs(roimage_centerbot_x - centroid_yellow[0])

        # Output which lane is closer
        if distance_to_white < distance_to_yellow:
            print("Robot is closer to the white lane.")
            return "white"
        else:
            print("Robot is closer to the yellow lane.")
            return "yellow"

        '''
                    

    def find_centroid(self, contours):
        """Find the centroid of the largest contour."""
        if not contours:
            return None
        
        # Sort contours by area and use the largest one
        largest_contour = max(contours, key=cv2.contourArea)
        moments = cv2.moments(largest_contour)
        
        if moments['m00'] != 0:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            return (cx, cy)
        return None





    def apply_p_controller(self, lane_center, image):
        """Applies P control to follow the lane smoothly while moving forward."""
        _, image_width, _ = image.shape
        image_center = image_width // 2
        error = lane_center - image_center  # Error: difference from the center

        # Proportional control calculation
        control = self.Kp * error  # Positive to steer correctly

        # Ensure a forward base speed
        base_speed = 0.2  # Adjust this based on your robot's dynamics

        print(f"control: {control}")

        # Compute left and right wheel speeds
        vel_left = base_speed + control
        vel_right = base_speed - control

        # Clamp values between -1 and 1
        vel_left = max(min(vel_left, 1.0), -1.0)
        vel_right = max(min(vel_right, 1.0), -1.0)

        print(f"image center: {image_center}, lane center: {lane_center}, Error: {error}, left: {vel_left}, right: {vel_right}")


        # Check if 350 ticks have been reached
        if self._ticks_left is not None and self._ticks_right is not None:
            ticks_travelled_left = abs(self._ticks_left - self._initial_ticks_left)
            ticks_travelled_right = abs(self._ticks_right - self._initial_ticks_right)

            if ticks_travelled_left >= 850 or ticks_travelled_right >= 850:
                rospy.loginfo_once("Stopping robot after 350 ticks")
                vel_left = 0.0
                vel_right = 0.0

        # Publish the velocity command
        cmd = WheelsCmdStamped()
        cmd.vel_left = vel_left
        cmd.vel_right = vel_right
        self.publisher.publish(cmd)



if __name__ == '__main__':
    # create the node
    node = ControllerNode(node_name='controller_node')
    # keep spinning
    rospy.spin()
