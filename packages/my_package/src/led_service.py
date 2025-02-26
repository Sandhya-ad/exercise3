#!/usr/bin/env python3

from duckietown_msgs.msg import LEDPattern  
from std_msgs.msg import ColorRGBA
import rospy


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

