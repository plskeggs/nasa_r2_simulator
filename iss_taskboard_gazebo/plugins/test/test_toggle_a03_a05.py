#!/usr/bin/env python
##
# Copyright (C) 2013 TopCoder Inc., All Rights Reserved.
#
# @author KennyAlive
# @version 1.0
#

## Tests for safe toggles A03/A05

PKG = 'gazebo_taskboard'
NAME = 'test_toggle_a03_a05'

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

toggle_a03_state = 'DOWN'
toggle_a05_state = 'DOWN'
toggle_a03_led_state = 'OFF'
toggle_a05_led_state = 'OFF'

def onMessage(message):
    global toggle_a03_state
    if toggle_a03_state != message.A03_TOGGLE:
        rospy.loginfo("Toggle A03: " + message.A03_TOGGLE)
    toggle_a03_state = message.A03_TOGGLE

    global toggle_a05_state
    if toggle_a05_state != message.A05_TOGGLE:
        rospy.loginfo("Toggle A05: " + message.A05_TOGGLE)
    toggle_a05_state = message.A05_TOGGLE

    global toggle_a03_led_state
    if toggle_a03_led_state != message.A03_LED:
        rospy.loginfo("Toggle A03 LED: " + message.A03_LED)
    toggle_a03_led_state = message.A03_LED

    global toggle_a05_led_state
    if toggle_a05_led_state != message.A05_LED:
        rospy.loginfo("Toggle A05 LED: " + message.A05_LED)
    toggle_a05_led_state = message.A05_LED


class ToggleA03A05Test(unittest.TestCase):
    def __init__(self, *args):
       super(ToggleA03A05Test, self).__init__(*args)

    def test_toggle_a03_a05(self):
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

            # Try to move up A03 toggle without pulling out. We should fail.
            manipulateSafeToggle(0, 1, deg2rad(40), 1)
            wait()
            assert toggle_a03_state == 'DOWN'
            assert toggle_a03_led_state == 'OFF'
            rospy.loginfo("Case 1 Done.")
            
            # Similar with A05 but at first pull out it. The operation should be successful.
            manipulateSafeToggle(2, 0, 4, 2)
            manipulateSafeToggle(2, 1, deg2rad(40), 1)
            wait(4)
            assert toggle_a05_state == 'UP'
            assert toggle_a05_led_state == 'ON'
            rospy.loginfo("Case 2 Done.")

            # Pull out A03 (OUT state) then release (again DOWN)
            manipulateSafeToggle(0, 0, 4, 5)
            wait(1)
            assert toggle_a03_state == 'OUT'
            assert toggle_a03_led_state == 'OFF'
            wait(5)
            assert toggle_a03_state == 'DOWN'
            assert toggle_a03_led_state == 'OFF'
            rospy.loginfo("Case 3 Done.")

             # Pull out A03 and move on small angle up (not sufficient for UP state)
            manipulateSafeToggle(0, 0, 4, 2)
            manipulateSafeToggle(0, 1, deg2rad(15), 1)
            wait(4)
            assert toggle_a03_state == 'DOWN'
            assert toggle_a03_led_state == 'OFF'
            rospy.loginfo("Case 4 Done.")
            
            # Finally put A03 to UP state
            manipulateSafeToggle(0, 0, 4, 2)
            manipulateSafeToggle(0, 1, deg2rad(40), 1)
            wait(4)
            assert toggle_a03_state == 'UP'
            assert toggle_a03_led_state == 'ON'
            rospy.loginfo("Case 5 Done.")
            
            # Put A05 into OUT state and check it (in the upper position)
            manipulateSafeToggle(2, 0, 5, 5)
            wait(1)
            assert toggle_a05_state == 'OUT'
            assert toggle_a05_led_state == 'OFF'
            rospy.loginfo("Case 6 Done.")
            
            # Move A05 on small angle not sufficient for DOWN position
            manipulateSafeToggle(2, 0, 4, 2)
            manipulateSafeToggle(2, 1, deg2rad(-15), 1)
            wait(4)
            assert toggle_a05_state == 'UP'
            assert toggle_a05_led_state == 'ON'
            rospy.loginfo("Case 7 Done.")
            
            # Move A05 to down position
            manipulateSafeToggle(2, 0, 4, 2)
            manipulateSafeToggle(2, 1, deg2rad(-40), 1)
            wait(4)
            assert toggle_a05_state == 'DOWN'
            assert toggle_a05_led_state == 'OFF'
            rospy.loginfo("Case 8 Done.")
            
            ## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            # Turn off the power
            manipulatePowerSwitch(deg2rad(-30), 1)
            wait()
            # Close power cover
            manipulatePowerCover(deg2rad(-50), 1)
            wait()
            ## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            
            # Check leds
            assert toggle_a03_led_state == 'OFF'
            assert toggle_a05_led_state == 'OFF'
            rospy.loginfo("Case 9 Done.")

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == '__main__':
    print "Waiting for test to start at time "
    rostest.run(PKG, 'test_toggle_a03_a05', ToggleA03A05Test)
