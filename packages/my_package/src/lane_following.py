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



class LaneFollowing(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LaneFollowing, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
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

        # control mode
        #self._controller_mode = "p"
        #self._controller_mode = "pd"
        self._controller_mode = "pid"



        # variables for Controller
        # use of self.Kp = 0.001 for p controller
        # use self.kp = 0.003 for pd controller
        # 0.0013
        self.Kp = 0.0015   # Tune this value absed on testing ask chatGPT more about it how increase 
        # and decrease affect the movement
        # use self.Kd = 0.005 for pd controller
        self.Kd = 0.005
        self.prev_error = 0
        self.Ki = 0.001
        self.integral = 0
        self.max_integral = 80
        # 0.09
        self.slow_speed = 0.13
        self.normal_speed = 0.2
        self.curve_threshold = 50

         # Variables for image processing optimization
        self.last_processed_image = None  # Stores the last cropped image
        # use 5 for self.kp
        # use 5 for self.kd
        self.difference_threshold = 5    # Adjust threshold as necessary
        self.threshold_count = 0
        
        

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
                    self.threshold_count += 1
                    if self.threshold_count < 11:
                        print("Not enough")
                        return
            
            print("reseting")
            self.threshold_count = 0

            # Update the last processed image (copy to avoid reference issues)
            self.last_processed_image = bottom_half.copy()


            # undistort the image
            undistorted_image = self.undistort_image(bottom_half)

            # Detect yellow dotted lane
            yellow_lanes_masking = self.detect_yellow_dotted_lane(undistorted_image)

            # detect white solid line
            white_lane_masking = self.detect_white_solid_lane(undistorted_image)

            # combine both lane masking
            #Combined_mask = cv2.bitwise_or(yellow_lanes_masking, white_lane_masking)

            #processed_msg_yellow = self._bridge.cv2_to_imgmsg(yellow_lanes_masking)
            #processed_msg_white = self._bridge.cv2_to_imgmsg(white_lane_masking)
            #processed_msg_both = self._bridge.cv2_to_compressed_imgmsg(undistorted_image)

            # Calculate the lane center
            lane_center = self.compute_lane_center(yellow_lanes_masking, white_lane_masking, undistorted_image)

            # Apply P controller to adjust steering
            if lane_center != None:
                if self._controller_mode == "p":
                    self.apply_p_controller(lane_center, undistorted_image)
                elif self._controller_mode == "pd":
                    self.apply_pd_controller(lane_center, undistorted_image)
                elif self._controller_mode == "pid":
                    self.apply_pid_controller(lane_center, undistorted_image)
            else:
                print(f"lane center: {lane_center} ############################################################################################")


            #self.pub_yellow.publish(processed_msg_yellow)
            #self.pub_white.publish(processed_msg_white)
            #self.pub_both.publish(processed_msg_both)




        

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
            print("None")
            return None  # Default to image center to prevent extreme corrections

        if centroid_white is None:
            lane_center = centroid_yellow[0]  # Use only yellow lane
        elif centroid_yellow is None:
            lane_center = centroid_white[0]  # Use only white lane
        else:
            lane_center = (centroid_yellow[0] + centroid_white[0]) // 2  # Average both lanes

        return lane_center  # Return the x-coordinate of the lane center
                    

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

        """Applies PD control to follow the lane smoothly while moving forward."""
        # Get the center of the image
        _, image_width, _ = image.shape
        image_center = image_width // 2

        # Calculate the error between the lane center and the image center
        error = lane_center - image_center  # Error: difference from the center

        # Calculate the derivative of the error.
        # Here, we assume a constant time interval between updates.
        # Make sure that self.prev_error is initialized (e.g., to 0) in your __init__ method.
        derivative = error - self.prev_error

        # Update the previous error for the next control cycle
        self.prev_error = error

        # PD control calculation: combining proportional and derivative actions.
        control = self.Kp * error + self.Kd * derivative

        # Adjust forward speed: if the error exceeds the curve threshold, use a slower speed
        if abs(error) > self.curve_threshold:
            base_speed = self.slow_speed
        else:
            base_speed = self.normal_speed

        print(f"control: {control}")

        # Compute left and right wheel speeds using the control signal
        vel_left = base_speed + control
        vel_right = base_speed - control

        # Clamp the velocity values between -1 and 1
        vel_left = max(min(vel_left, 1.0), -1.0)
        vel_right = max(min(vel_right, 1.0), -1.0)

        print(f"image center: {image_center}, lane center: {lane_center}, Error: {error}, Derivative: {derivative}, left: {vel_left}, right: {vel_right}")

        # Check if 850 ticks have been reached to potentially stop the robot
        if self._ticks_left is not None and self._ticks_right is not None:
            ticks_travelled_left = abs(self._ticks_left - self._initial_ticks_left)
            ticks_travelled_right = abs(self._ticks_right - self._initial_ticks_right)
            
            # If either wheel has reached 850 ticks, stop the robot
            if ticks_travelled_left >= 1500 or ticks_travelled_right >= 1500:
                rospy.loginfo_once("Stopping robot after 900 ticks")
                vel_left = 0.0
                vel_right = 0.0

        # Publish the velocity command
        cmd = WheelsCmdStamped()
        cmd.vel_left = vel_left
        cmd.vel_right = vel_right
        self.publisher.publish(cmd)


    def apply_pid_controller(self, lane_center, image):
        """Applies PID control with anti-windup to follow the lane smoothly.
        
        This version is tuned for an oval lane where curves are important.
        It slows down when a large lateral error is detected (indicating a curve).
        
        Ensure that the following parameters are initialized in __init__:
        - self.Kp, self.Ki, self.Kd: PID gains.
        - self.max_integral: Maximum integral value (e.g., 50).
        - self.curve_threshold: Error threshold beyond which the robot is considered to be in a curve.
        - self.normal_speed: Forward speed in straight sections (e.g., 0.2).
        - self.slow_speed: Reduced speed for curves (e.g., 0.1).
        """
        # Get image dimensions and compute image center
        _, image_width, _ = image.shape
        image_center = image_width // 2

        # Calculate the error between the detected lane center and the image center
        error = lane_center - image_center

        # DDerivative: difference between current and previous error
        derivative = error - self.prev_error

        # Update the previous error for the next cycle
        self.prev_error = error

        '''        
        # Choose dynamic max_integral based on whether the error indicates a curve
        if abs(error) > self.curve_threshold:
            self.max_integral = 80
        else:
            self.max_integral = 23
        '''
    
        #Update the integral term (using dt=1 for simplicity)
        self.integral += error
        
        # Anti-windup: Clamp integral value to the range [0, max_integral]
        if self.integral >= self.max_integral:
            self.integral -= error
            if self.integral <= 75:
                self.integral *= 0.80
        elif self.integral < 0:
            self.integral = 0
                

            
        
        print(f"Error: {error}, Integral: {self.integral}, Derivative: {derivative}")

                
            

        # PID control: combine proportional, integral, and derivative terms
        control = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Adjust base speed depending on the error (for better performance on curves)
        # When the error exceeds the curve_threshold, use a slower speed
        if abs(error) > self.curve_threshold:
            base_speed = self.slow_speed
        else:
            base_speed = self.normal_speed


        #print(f"control: {control}, max integral: {self.max_integral}")

        # Compute left and right wheel speeds (differential drive control)
        vel_left = base_speed + control
        vel_right = base_speed - control

        # Clamp the wheel velocities between -1.0 and 1.0
        vel_left = max(min(vel_left, 1.0), -1.0)
        vel_right = max(min(vel_right, 1.0), -1.0)

        print(f"left: {vel_left}, right: {vel_right}")


        # Check if 850 ticks have been reached to potentially stop the robot
        if self._ticks_left is not None and self._ticks_right is not None:
            ticks_travelled_left = abs(self._ticks_left - self._initial_ticks_left)
            ticks_travelled_right = abs(self._ticks_right - self._initial_ticks_right)
            
            # If either wheel has reached 850 ticks, stop the robot
            if ticks_travelled_left >= 4050 or ticks_travelled_right >= 4050:
                rospy.loginfo_once("Stopping robot after 4000 ticks")
                vel_left = 0.0
                vel_right = 0.0


        # Publish the velocity command
        cmd = WheelsCmdStamped()
        cmd.vel_left = vel_left
        cmd.vel_right = vel_right
        self.publisher.publish(cmd)






if __name__ == '__main__':
    # create the node
    node = LaneFollowing(node_name='lane_following_node')
    # keep spinning
    rospy.spin()
