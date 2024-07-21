#!/usr/bin/env python3

import rospy
from rover6_dronecan.srv import SetLED, SetLEDRequest, SetLEDResponse
from std_msgs.msg import ColorRGBA

if __name__ == "__main__":
    rospy.init_node('rover6_dronecan_led_test')
    rospy.loginfo("Testing leds")
    leds_service = rospy.ServiceProxy('/rover6/set_led', SetLED)
    leds_service(SetLEDRequest(channel=0, color1=ColorRGBA(1, 0, 0, 0), color2=ColorRGBA(0, 0, 0, 0), effect=5))
    leds_service(SetLEDRequest(channel=1, color1=ColorRGBA(0, 1, 0, 0), color2=ColorRGBA(0, 0, 0, 0), effect=5))
    leds_service(SetLEDRequest(channel=2, color1=ColorRGBA(0, 0, 1, 0), color2=ColorRGBA(0, 0, 0, 0), effect=5))
