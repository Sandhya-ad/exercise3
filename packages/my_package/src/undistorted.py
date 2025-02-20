#!/usr/bin/env python3


import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
import numpy as np

import cv2
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._undistorted_image_topic = f"/{self._vehicle_name}/camera_undistorted/image/compressed"
        self._camera_info_topic = f"/{self._vehicle_name}/camera_node/camera_info"  

        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        # self._window = "camera-reader"
        #cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
         # Subscribe to the camera topic
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        self.sub_info = rospy.Subscriber(self._camera_info_topic, CameraInfo, self.camera_info_callback)

        # Publisher for the processed image
        self.pub = rospy.Publisher(self._undistorted_image_topic, CompressedImage, queue_size=1)

        # Processing frequency
        self.rate = rospy.Rate(3)  # 3 Hz (3-5 frames per second)

    
    def camera_info_callback(self, msg):
        """ Callback to receive camera intrinsic parameters. """
        self.camera_matrix = np.array(msg.K).reshape((3, 3))
        self.dist_coeffs = np.array(msg.D)
        rospy.loginfo("Camera intrinsic parameters received.")
    def undistort_image(self, image):
        """ Applies undistortion using stored camera parameters. """
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logwarn("Camera parameters not received yet!")
            return image  # Return original if no parameters

        h, w = image.shape[:2]
        
        # Get optimal new camera matrix
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), alpha=0.0, newImgSize=(w, h)
        )

        # Apply undistortion
        undistorted = cv2.undistort(image, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)

        # Crop out black borders (use ROI)
        x, y, w, h = roi
        if w > 0 and h > 0:  # Ensure valid ROI
            undistorted = undistorted[y:y+h, x:x+w]

        return undistorted


    def callback(self, msg):
        """ Process the incoming image: undistort and publish. """
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Undistort the image
            undistorted_image = self.undistort_image(image)
            h, w = undistorted_image.shape[:2]
            resized_image = undistorted_image
	
            # Scaling Up the image 1.2 times by specifying both scaling factors
            scale_up_x = 1.5
            scale_up_y = 1.5

            
            #scaled_f_down = cv2.resize(resized_image, None, fx= scale_down, fy= scale_down, interpolation= cv2.INTER_LINEAR)
            resized_image = cv2.resize(resized_image, None, fx= scale_up_x, fy= scale_up_y, interpolation= cv2.INTER_LINEAR)


            # resized_image = cv2.resize(undistorted_image, (w//2, h//2), interpolation=cv2.INTER_AREA)   #resized image
            resized_image = cv2.GaussianBlur(resized_image,(5,5),0)     #resized and blurres image


            # Encode back to CompressedImage
            _, img_encoded = cv2.imencode('.jpg', resized_image)
            processed_msg = CompressedImage()
            processed_msg.header = msg.header
            processed_msg.format = "jpeg"
            processed_msg.data = np.array(img_encoded).tobytes()

            # Publish undistorted image
            self.pub.publish(processed_msg)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()