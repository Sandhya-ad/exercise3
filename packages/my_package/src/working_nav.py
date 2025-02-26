#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped
from led_service import LEDBlinker

# Wheel speed settings
DEFAULT_SPEED_LEFT = 0.3  
DEFAULT_SPEED_RIGHT = 0.3  
WHEEL_RADIUS = 0.031  # meters (example value)
TICKS_PER_REVOLUTION = 135  # Example value
WHEEL_CIRCUMFERENCE = 2 * 3.1416 * WHEEL_RADIUS  # meters per wheel revolution
"""
navigation with DTROS so that it can run on it's own for debugging and stuff
"""
class DTrajectory(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(DTrajectory, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        self._wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"

        # Initialize LED controller
        self.led_controller = LEDBlinker()

        # Subscribe to wheel encoders
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        self.publisher = rospy.Publisher(self._wheels_topic, WheelsCmdStamped, queue_size=1)

        # Wheel encoder tracking
        self._ticks_left = None
        self._ticks_right = None
        self._initial_ticks_left = None
        self._initial_ticks_right = None

    def callback_left(self, data):
        if self._initial_ticks_left is None:
            self._initial_ticks_left = data.data
        self._ticks_left = data.data

    def callback_right(self, data):
        if self._initial_ticks_right is None:
            self._initial_ticks_right = data.data
        self._ticks_right = data.data

    def move_straight(self, distance):
        """ Moves in a straight line for a specified distance in meters. """
        rospy.loginfo(f"Moving straight for {distance} meters.")
        self.led_controller.set_led_color("blue")  # Set LED to blue while moving

        # Convert distance to encoder ticks
        ticks_to_move = int((distance / WHEEL_CIRCUMFERENCE) * TICKS_PER_REVOLUTION)

        # Reset initial encoder values
        self._initial_ticks_left = self._ticks_left
        self._initial_ticks_right = self._ticks_right

        # Move while tracking encoder ticks
        command = WheelsCmdStamped(vel_left=DEFAULT_SPEED_LEFT, vel_right=DEFAULT_SPEED_RIGHT)
        while (self._ticks_left - self._initial_ticks_left) < ticks_to_move and \
              (self._ticks_right - self._initial_ticks_right) < ticks_to_move:
            self.publisher.publish(command)
            rospy.sleep(0.1)

        self.stop(1)

    def move_curve_right(self):
        """ Moves in a curve through 90 degrees to the right. """
        rospy.loginfo("Curving right for 90 degrees.")
        self.led_controller.set_led_color("yellow")

        # Right wheel slower, left wheel faster
        command = WheelsCmdStamped(vel_left=0.4, vel_right=0.2)

        # Reset encoder values
        self._initial_ticks_left = self._ticks_left
        self._initial_ticks_right = self._ticks_right

        # Move until wheels turn required number of ticks
        while (self._ticks_left - self._initial_ticks_left) < 400 or (self._ticks_right - self._initial_ticks_right) < 120:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(command)
            


        self.stop(1)

    def move_curve_left(self):
        """ Moves in a curve through 90 degrees to the left. """
        rospy.loginfo("Curving left for 90 degrees.")
        self.led_controller.set_led_color("yellow")

        # Left wheel slower, right wheel faster
        command = WheelsCmdStamped(vel_left=0.2, vel_right=0.4)

        # Reset encoder values
        self._initial_ticks_left = self._ticks_left
        self._initial_ticks_right = self._ticks_right

        # Move until wheels turn required number of ticks
        while (self._ticks_left - self._initial_ticks_left) < 120 or \
            (self._ticks_right - self._initial_ticks_right) < 350:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(command)

        self.stop(1)


    def stop(self, duration):
        """ Stops the bot for a specified duration. """
        rospy.loginfo(f"Stopping for {duration} seconds.")
        self.led_controller.set_led_color("red")  # Set LED to red when stopped

        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        rospy.sleep(duration)

    def run(self):
        """ Runs the navigation sequence. """
        self.led_controller.set_led_color("white")  # Initial state

        # Ensure encoder values are initialized
        while self._initial_ticks_left is None or self._initial_ticks_right is None:
            rospy.sleep(0.1)

        rospy.loginfo("Starting autonomous navigation sequence...")

        # Sequence of movements
        self.move_straight(1.2)  # Move straight for 1.2 meters
        #self.move_curve_left()  # Curve right 90 degrees
        # self.move_straight(0.92)  # Move forward 0.92 meters
        # self.move_curve_right()  # Curve right 90 degrees
        # self.move_straight(0.61)  # Move forward 0.61 meters
        # self.move_curve_right()  # Curve right 90 degrees
        # self.move_straight(0.92)  # Move forward 0.92 meters
        # self.move_curve_right()  # Curve right 90 degrees

        rospy.loginfo("Navigation complete. Stopping the bot.")
        self.stop(3)

if __name__ == '__main__':
    # Create the node
    trajectory_node = DTrajectory(node_name='d_trajectory')
    
    # Run the trajectory
    trajectory_node.run()
