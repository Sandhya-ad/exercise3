#!/usr/bin/env python3

from duckietown_msgs.msg import LEDPattern  
from std_msgs.msg import ColorRGBA
import rospy

"""
indices:
first: front left
second: back right
third: idk
fourth: back left
fifth is front right
"""


class LEDBlinker:
    def __init__(self):
        self.led_pub = rospy.Publisher('/csc22907/led_emitter_node/led_pattern', LEDPattern, queue_size=10)




        # Define color mappings
        self.colors = {
            "red": ColorRGBA(1.0, 0.0, 0.0, 1.0),
            "green": ColorRGBA(0.0, 1.0, 0.0, 1.0),
            "blue": ColorRGBA(0.0, 0.0, 1.0, 1.0),
            "yellow": ColorRGBA(1.0, 1.0, 0.0, 1.0),
            "purple": ColorRGBA(1.0, 0.0, 1.0, 1.0),
            "white": ColorRGBA(1.0, 1.0, 1.0, 1.0),    
        }




    def set_led_color(self, color_name):
        if color_name not in self.colors:
            rospy.logwarn(f"Invalid color: {color_name}")
            return

        color = self.colors[color_name]  # Convert string to ColorRGBA object
        led_msg = LEDPattern()
        led_msg.rgb_vals = [color] * 5  # Set all LEDs to the same color
        led_msg.frequency = 1.0  # Optional flashing effect
        led_msg.frequency_mask = [1, 1, 1, 1, 1]  # Apply frequency to all LEDs




        self.led_pub.publish(led_msg)
        rospy.loginfo(f"LED set to {color_name}")

        
    def set_led_color2(self, color_name, led_indices):  
        """
        Sets the LED color for specific LEDs.

        :param color_name: The name of the color (must be in self.colors).
        :param led_indices: List of LED indices (length 5, values 0 or 1) to change.
        """
        if color_name not in self.colors:
            rospy.logwarn(f"Invalid color: {color_name}")
            return

        color = self.colors[color_name]  # Get the ColorRGBA object
        led_msg = LEDPattern()

        # Initialize all LEDs as off (black)
        off_color = ColorRGBA(0.0, 0.0, 0.0, 1.0)
        led_msg.rgb_vals = [off_color] * 5  # Ensure correct array size (5 LEDs)

        # Set only the specified LEDs to the chosen color
        for i in range(5):  # Now use range(5)
            if led_indices[i] == 1:  
                led_msg.rgb_vals[i] = color  # Assign the ColorRGBA object

        led_msg.frequency = 0.0  # Solid color, no blinking
        led_msg.frequency_mask = [1 if led_indices[i] == 1 else 0 for i in range(5)]

        self.led_pub.publish(led_msg)
        rospy.loginfo(f"LEDs {led_indices} set to {color_name}")
