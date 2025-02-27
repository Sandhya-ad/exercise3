#!/usr/bin/env python3

import os
import rospy
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped
from led_service import LEDBlinker  # Import LED module

# Wheel parameters
DEFAULT_SPEED_LEFT = 0.31
DEFAULT_SPEED_RIGHT = 0.3
WHEEL_RADIUS = 0.033  # meters
TICKS_PER_REVOLUTION = 131 
WHEEL_CIRCUMFERENCE = 2 * 3.1416 * WHEEL_RADIUS  # meters per wheel revolution

#This works fine 
"""
Handles moving straight, curve right and curve left
DTROS not initialized so that it can be used in lane_based behaviour
"""

class Navigator:
    """ A class to control Duckiebot navigation movements. """

    def __init__(self):
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'duckiebot')
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        self._wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"

        self.led_controller = LEDBlinker()

        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        self.publisher = rospy.Publisher(self._wheels_topic, WheelsCmdStamped, queue_size=1)

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
        """ Moves the bot in a straight line for a given distance (meters). """
        rospy.loginfo(f"Moving straight for {distance} meters.")
        self.led_controller.set_led_color("blue")

        # Ensure encoders are initialized
        while self._ticks_left is None or self._ticks_right is None:
            rospy.logwarn("Waiting for encoder values to initialize...")
            rospy.sleep(0.1)

        ticks_to_move = int((distance / WHEEL_CIRCUMFERENCE) * (TICKS_PER_REVOLUTION-14))

        self._initial_ticks_left = self._ticks_left
        self._initial_ticks_right = self._ticks_right

        command = WheelsCmdStamped(vel_left=DEFAULT_SPEED_LEFT, vel_right=DEFAULT_SPEED_RIGHT)
        while (self._ticks_left - self._initial_ticks_left) < (ticks_to_move) and \
              (self._ticks_right - self._initial_ticks_right) < ticks_to_move:
            self.publisher.publish(command)
            rospy.sleep(0.1)

        self.stop(1)

    def move_curve_right(self):
        """ Moves the bot in a 90-degree right curve. """
        rospy.loginfo("Curving right for 90 degrees.")

        rospy.sleep(1)

        command = WheelsCmdStamped(vel_left=0.52, vel_right=0.2)

        self._initial_ticks_left = self._ticks_left
        self._initial_ticks_right = self._ticks_right

        while (self._ticks_left - self._initial_ticks_left) < 450 and \
              (self._ticks_right - self._initial_ticks_right) < 120:
            self.publisher.publish(command)
            rospy.sleep(0.1)

        self.stop(1)

    def move_curve_left(self):
        """ Moves the bot in a 90-degree left curve. """
        rospy.loginfo("Curving left for 90 degrees.")
        rospy.sleep(1)

        command = WheelsCmdStamped(vel_left=0.2, vel_right=0.5)

        self._initial_ticks_left = self._ticks_left
        self._initial_ticks_right = self._ticks_right

        while (self._ticks_left - self._initial_ticks_left) < 120 and \
              (self._ticks_right - self._initial_ticks_right) < 450:
            self.publisher.publish(command)
            rospy.sleep(0.1)

        self.stop(1)

    def stop(self, duration):
        """ Stops the bot for a specified duration. """
        rospy.loginfo(f"Stopping for {duration} seconds.")
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        rospy.sleep(duration)
