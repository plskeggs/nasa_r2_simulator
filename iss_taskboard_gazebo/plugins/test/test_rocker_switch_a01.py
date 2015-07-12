#!/usr/bin/env python
##
# Copyright (C) 2013 TopCoder Inc., All Rights Reserved.
#
# @author KennyAlive
# @version 1.0
#

## Tests for rocker switch A01

PKG = 'gazebo_taskboard'
NAME = 'test_rocker_switch_a01'

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

rocker_switch_state = 'CENTER'
led_top_state = 'OFF'
led_bottom_state = 'OFF'

def onMessage(message):
    global rocker_switch_state
    if rocker_switch_state != message.A01_ROCKER_SWITCH:
        rospy.loginfo("Rocker switch A01: " + message.A01_ROCKER_SWITCH)
    rocker_switch_state = message.A01_ROCKER_SWITCH
    
    global led_top_state
    if led_top_state != message.A01_ROCKER_LED_TOP:
        rospy.loginfo("Rocker LED TOP: " + message.A01_ROCKER_LED_TOP)
    led_top_state = message.A01_ROCKER_LED_TOP
        
    global led_bottom_state
    if led_bottom_state != message.A01_ROCKER_LED_BOTTOM:
        rospy.loginfo("Rocker LED BOTTOM: " + message.A01_ROCKER_LED_BOTTOM)
    led_bottom_state = message.A01_ROCKER_LED_BOTTOM


class RockerSwitchA01Test(unittest.TestCase):
    def __init__(self, *args):
       super(RockerSwitchA01Test, self).__init__(*args)

    def test_rocker_switch_a01(self):
        rospy.init_node(NAME, anonymous=True)

        rospy.Subscriber(TOPIC_NAME, TaskboardPanelA, onMessage)

        rospy.loginfo("Waiting for gazebo ...")
        rospy.wait_for_service(SERVICE_MANIPULATE_POWER_COVER)
        rospy.wait_for_service(SERVICE_MANIPULATE_POWER_SWITCH)
        rospy.wait_for_service(SERVICE_MANIPULATE_ROCKER_SWITCH)
        wait(GUI_WAIT_TIME)
        rospy.loginfo("Testing has started")

        try:
            manipulatePowerCover = rospy.ServiceProxy(SERVICE_MANIPULATE_POWER_COVER, ManipulatePowerCover)
            manipulatePowerSwitch = rospy.ServiceProxy(SERVICE_MANIPULATE_POWER_SWITCH, ManipulatePowerSwitch)
            manipulateRockerSwitchA01 = rospy.ServiceProxy(SERVICE_MANIPULATE_ROCKER_SWITCH, ManipulateRockerSwitch)

            # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            # Open power cover
            manipulatePowerCover(deg2rad(50), 1)
            wait()
            # Turn on power
            manipulatePowerSwitch(deg2rad(30), 1)
            wait()
            ## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            # Apply force and put rocker switch to UP position but do not stay in UP for too long
            manipulateRockerSwitchA01(10, 1)
            wait(4)
            assert rocker_switch_state == 'CENTER'
            assert led_top_state == 'OFF'
            assert led_bottom_state == 'OFF'
            rospy.loginfo("Case 1 Done.")
            
            # Apply force and put rocker switch to UP position and stay in UP for some time (apply force long enough)
            manipulateRockerSwitchA01(10, 8)
            wait(1)
            assert rocker_switch_state == 'UP'
            assert led_top_state == 'ON'
            assert led_bottom_state == 'OFF'
            rospy.loginfo("Case 2 Done.")
            
            # Apply force and put rocker switch to DOWN position but do not stay in DOWN for too long
            manipulateRockerSwitchA01(-10, 1)
            wait(4)
            assert rocker_switch_state == 'CENTER'
            assert led_top_state == 'OFF'
            assert led_bottom_state == 'OFF'
            rospy.loginfo("Case 3 Done.")

            # Apply force and put rocker switch to DOWN position and stay in DOWN for some time (apply force long enough)
            manipulateRockerSwitchA01(-10, 8)
            wait(1)
            assert rocker_switch_state == 'DOWN'
            assert led_top_state == 'OFF'
            assert led_bottom_state == 'ON'
            rospy.loginfo("Case 4 Done.")

            ## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            # Turn off the power
            manipulatePowerSwitch(deg2rad(-30), 1)
            wait()
            # Close power cover
            manipulatePowerCover(deg2rad(-50), 1)
            wait()
            ## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            # Apply force and put rocker switch to UP position and stay in UP for some time (apply force long enough)
            manipulateRockerSwitchA01(10, 8)
            wait(1)
            assert rocker_switch_state == 'UP'
            assert led_top_state == 'OFF'
            assert led_bottom_state == 'OFF'
            rospy.loginfo("Case 5 Done.")

            # Apply force and put rocker switch to DOWN position and stay in DOWN for some time (apply force long enough)
            manipulateRockerSwitchA01(-10, 8)
            wait(1)
            assert rocker_switch_state == 'DOWN'
            assert led_top_state == 'OFF'
            assert led_bottom_state == 'OFF'
            rospy.loginfo("Case 6 Done.")

            # Check the final state
            wait(9)
            assert rocker_switch_state == 'CENTER'
            assert led_top_state == 'OFF'
            assert led_bottom_state == 'OFF'
            rospy.loginfo("Case 7 Done.")

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        time.sleep(1.0)

if __name__ == '__main__':
    print "Waiting for test to start at time "
    rostest.run(PKG, 'test_rocker_switch_a01', RockerSwitchA01Test)
