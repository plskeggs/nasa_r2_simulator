#!/usr/bin/env python
##
# Copyright (C) 2013 TopCoder Inc., All Rights Reserved.
#
# @author KennyAlive
# @version 1.0
#

## Tests for power switch

PKG = 'gazebo_taskboard'
NAME = 'test_power_switch'

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

power_cover_state = 'DOWN'
power_switch_state = 'DOWN'
power_led_state = 'OFF'

def onMessage(msg):
    global power_cover_state
    power_cover_state = msg.PANEL_POWER_COVER

    global power_switch_state
    if power_switch_state != msg.PANEL_POWER_SWITCH:
        rospy.loginfo("Power switch state: " + msg.PANEL_POWER_SWITCH)
    power_switch_state = msg.PANEL_POWER_SWITCH

    global power_led_state
    if power_led_state != msg.PANEL_POWER_LED:
        rospy.loginfo("Power LED state: " + msg.PANEL_POWER_LED)
    power_led_state = msg.PANEL_POWER_LED
    

class PowerSwitchTest(unittest.TestCase):
    def __init__(self, *args):
       super(PowerSwitchTest, self).__init__(*args)

    def test_power_switch(self):
        rospy.init_node(NAME, anonymous=True)

        rospy.Subscriber(TOPIC_NAME, TaskboardPanelA, onMessage)

        rospy.loginfo("Waiting for gazebo ...")
        rospy.wait_for_service(SERVICE_MANIPULATE_POWER_COVER)
        rospy.wait_for_service(SERVICE_MANIPULATE_POWER_SWITCH)
        wait(GUI_WAIT_TIME)
        rospy.loginfo("Testing has started")

        try:
            manipulatePowerCover = rospy.ServiceProxy(SERVICE_MANIPULATE_POWER_COVER, ManipulatePowerCover)
            manipulatePowerSwitch = rospy.ServiceProxy(SERVICE_MANIPULATE_POWER_SWITCH, ManipulatePowerSwitch)

            manipulatePowerCover(deg2rad(50), 1)
            wait()
            assert power_cover_state == 'UP'
            rospy.loginfo("Case 1 Done.")

            manipulatePowerSwitch(deg2rad(30), 1)
            wait()
            assert power_switch_state == 'UP'
            rospy.loginfo("Case 2 Done.")
            
            manipulatePowerSwitch(deg2rad(-10), 0.5)
            wait()
            assert power_switch_state == 'UP'
            rospy.loginfo("Case 3 Done.")
            
            manipulatePowerSwitch(deg2rad(-20), 0.5)
            wait()
            assert power_switch_state == 'UP'
            rospy.loginfo("Case 4 Done.")
            
            manipulatePowerSwitch(deg2rad(-27), 0.75)
            wait()
            assert power_switch_state == 'DOWN'
            rospy.loginfo("Case 5 Done.")
            
            manipulatePowerSwitch(deg2rad(15), 0.3)
            wait()
            assert power_switch_state == 'DOWN'
            rospy.loginfo("Case 6 Done.")
            
            manipulatePowerSwitch(deg2rad(35), 0.75)
            wait()
            assert power_switch_state == 'UP'
            rospy.loginfo("Case 7 Done.")
            
            manipulatePowerSwitch(deg2rad(-35), 0.75)
            wait()
            assert power_switch_state == 'DOWN'
            rospy.loginfo("Case 8 Done.")

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        time.sleep(3.0)

if __name__ == '__main__':
    print "Waiting for test to start at time "
    rostest.run(PKG, 'test_power_switch', PowerSwitchTest)
