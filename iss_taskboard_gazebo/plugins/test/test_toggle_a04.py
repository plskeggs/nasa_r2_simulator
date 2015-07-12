#!/usr/bin/env python
##
# Copyright (C) 2013 TopCoder Inc., All Rights Reserved.
#
# @author KennyAlive
# @version 1.0
#

## Tests for toggle a04

PKG = 'gazebo_taskboard'
NAME = 'test_toggle_a04'

import math
import roslib
roslib.load_manifest(PKG)

import sys, unittest
import os, os.path, threading, time
import rospy, rostest

from std_msgs.msg import String
from helper import *
from gazebo_taskboard.msg import *
from gazebo_taskboard.srv import *

toggle_a04_state = 'DOWN'
toggle_a04_led_top_state = 'OFF'
toggle_a04_led_bottom_state = 'OFF'

def onMessage(message):
    global toggle_a04_state
    if toggle_a04_state != message.A04_TOGGLE:
        rospy.loginfo("Toggle A03: " + message.A04_TOGGLE)
    toggle_a04_state = message.A04_TOGGLE
    
    global toggle_a04_led_top_state
    if toggle_a04_led_top_state != message.A04_LED_TOP:
        rospy.loginfo("Toggle A04 TOP LED: " + message.A04_LED_TOP)
    toggle_a04_led_top_state = message.A04_LED_TOP
    
    global toggle_a04_led_bottom_state
    if toggle_a04_led_bottom_state != message.A04_LED_BOTTOM:
        rospy.loginfo("Toggle A04 BOTTOM LED: " + message.A04_LED_BOTTOM)
    toggle_a04_led_bottom_state = message.A04_LED_BOTTOM
    

class ToggleA04Test(unittest.TestCase):
    def __init__(self, *args):
       super(ToggleA04Test, self).__init__(*args)

    def test_toggle_a04(self):
        rospy.init_node(NAME, anonymous=True)

        rospy.Subscriber(TOPIC_NAME, TaskboardPanelA, onMessage)

        rospy.loginfo("Waiting for gazebo ...")
        rospy.wait_for_service(SERVICE_MANIPULATE_POWER_COVER)
        rospy.wait_for_service(SERVICE_MANIPULATE_POWER_SWITCH)
        rospy.wait_for_service(SERVICE_MANIPULATE_SAFE_TOGGLE)
        wait(GUI_WAIT_TIME)
        rospy.loginfo("Testing has started")

        try:
            manipulatePowerCover = rospy.ServiceProxy(SERVICE_MANIPULATE_POWER_COVER, ManipulatePowerCover)
            manipulatePowerSwitch = rospy.ServiceProxy(SERVICE_MANIPULATE_POWER_SWITCH, ManipulatePowerSwitch)
            manipulateSafeToggle = rospy.ServiceProxy(SERVICE_MANIPULATE_SAFE_TOGGLE, ManipulateSafeToggle)

            # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            # Open power cover
            manipulatePowerCover(deg2rad(50), 1)
            wait()
            # Turn on power
            manipulatePowerSwitch(deg2rad(30), 1)
            wait()
            ## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            
            # Try to move up A04 toggle without pulling out. We should fail.
            manipulateSafeToggle(1, 1, deg2rad(40), 1)
            wait()
            assert toggle_a04_state == 'DOWN'
            assert toggle_a04_led_top_state == 'OFF'
            assert toggle_a04_led_bottom_state == 'ON'
            rospy.loginfo("Case 1 Done.")

            # Perform successful switch to UP state
            manipulateSafeToggle(1, 0, 10, 2)
            manipulateSafeToggle(1, 1, deg2rad(40), 1)
            wait(4)
            assert toggle_a04_state == 'UP'
            assert toggle_a04_led_top_state == 'ON'
            assert toggle_a04_led_bottom_state == 'OFF'
            rospy.loginfo("Case 2 Done.")

            # Go to the center position
            manipulateSafeToggle(1, 0, 10, 2)
            manipulateSafeToggle(1, 1, deg2rad(-26), 1)
            wait(4)
            assert toggle_a04_state == 'CENTER'
            assert toggle_a04_led_top_state == 'OFF'
            assert toggle_a04_led_bottom_state == 'OFF'
            rospy.loginfo("Case 3 Done.")

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        time.sleep(3.0)

if __name__ == '__main__':
    print "Waiting for test to start at time "
    rostest.run(PKG, 'test_toggle_a04', ToggleA04Test)
