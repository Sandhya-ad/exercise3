#!/usr/bin/env python3

import rospy
import time
from led_service import LEDBlinker  # Ensure this file is named `led_blinker.py` in the same directory


def main():
    rospy.init_node('led_tester', anonymous=True)
    blinker = LEDBlinker()

    rate = rospy.Rate(1)  # 1 Hz loop rate

    while not rospy.is_shutdown():
        # Test each LED individually to find their correct indices
        blinker.set_led_color2("red", [1, 0, 0, 0, 0])  # Should light up LED 0
        rospy.sleep(1)

        blinker.set_led_color2("green", [0, 1, 0, 0, 0])  # Should light up LED 1
        rospy.sleep(1)

        blinker.set_led_color2("blue", [0, 0, 1, 0, 0])  # Should light up LED 2
        rospy.sleep(1)

        blinker.set_led_color2("yellow", [0, 0, 0, 1, 0])  # Should light up LED 3
        rospy.sleep(1)

        blinker.set_led_color2("white", [0, 0, 0, 0, 1])  # Should light up LED 4
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
