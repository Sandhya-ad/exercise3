#!/usr/bin/env python3

import rospy
from lane_detection import LaneDetectionNode  # Import color detection
from led_service import LEDBlinker  # Import LED control
from auto_nav_func import Navigator  # Import navigation functions
from duckietown.dtros import DTROS, NodeType  # Import DTROS


""""
Main function for the lane-based0 behaviour movement
calls led, auto-nav and color detection in one code

""""

class LaneBehavior(DTROS):  # Inherit from DTROS instead of manually calling rospy.init_node()
    def __init__(self):
        super(LaneBehavior, self).__init__(node_name="lane_behavior_node", node_type=NodeType.CONTROL)

        self.lane_detector = LaneDetectionNode(node_name="lane_detection_node")
        self.led_controller = LEDBlinker()
        self.navigator = Navigator()

        rospy.loginfo("Lane Behavior Node Initialized")
    def execute_behavior(self):
        """Detects lane color and triggers appropriate movement behavior ONCE, then stops."""
        rate = rospy.Rate(1)  # Check lane every 0.5s

        detected_color = None
        while detected_color is None and not rospy.is_shutdown():
            detected_color, _ = self.lane_detector.detect_lanes() 
            rospy.loginfo("Waiting for lane detection...")
            rate.sleep()

        if detected_color == "blue":
            self.handle_blue_lane()
        elif detected_color == "red":
            self.handle_red_lane()
        elif detected_color == "green":
            self.handle_green_lane()

        rospy.loginfo("Finished execution. Stopping node.")
        rospy.signal_shutdown("Task completed")


    def handle_blue_lane(self):
        """Behavior when approaching a blue line."""
        rospy.loginfo("Detected BLUE lane. Executing behavior.")
        self.navigator.stop(2)

        self.navigator.move_straight(0.3)
        self.navigator.stop(4)  # Stop for 3 seconds
        self.led_controller.set_led_color("blue")  # Signal right
        self.navigator.move_curve_right()  # Move in a right curve

    def handle_red_lane(self):
        """Behavior when approaching a red line."""
        rospy.loginfo("Detected RED lane. Executing behavior.")
        self.navigator.move_straight(0.3)
        self.navigator.stop(3)  # Stop for 3 seconds
        self.navigator.move_straight(0.35)  # Move straight for 30 cm

    def handle_green_lane(self):
        """Behavior when approaching a green line."""
        rospy.loginfo("Detected GREEN lane. Executing behavior.")
        self.navigator.move_straight(0.3)
        self.navigator.stop(3)  # Stop for 3 seconds
        self.led_controller.set_led_color("green")  # Signal left
        self.navigator.move_curve_left()  # Move in a left curve

if __name__ == "__main__":
    behavior = LaneBehavior()
    behavior.execute_behavior()
