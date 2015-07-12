#!/usr/bin/env python
##
# Copyright (C) 2013 TopCoder Inc., All Rights Reserved.
#
# @author KennyAlive
# @version 1.0
#

## Test for power cover

PKG = 'iss_taskboard_gazebo'
NAME = 'test_power_cover'

import math

import sys, unittest
import os, os.path, threading, time
import rospy, rostest

from std_msgs.msg import String
from helper import *
from iss_taskboard_gazebo.msg import *
from iss_taskboard_gazebo.srv import *

last_state = 'DOWN'

def onMessage(msg):
    global last_state
    if last_state != msg.PANEL_POWER_COVER:
        rospy.loginfo("NEW STATE: " + msg.PANEL_POWER_COVER)
    last_state = msg.PANEL_POWER_COVER

class PowerCoverTest(unittest.TestCase):
    def __init__(self, *args):
       super(PowerCoverTest, self).__init__(*args)
       
    def test_power_cover(self):
        rospy.init_node(NAME, anonymous=True)

        rospy.Subscriber(TOPIC_NAME, TaskboardPanelA, onMessage)

        print "Waiting for gazebo ..."
        rospy.wait_for_service(SERVICE_MANIPULATE_POWER_COVER)
        wait(GUI_WAIT_TIME)
        print "Test started."

        try:
            manipulatePowerCover = rospy.ServiceProxy(SERVICE_MANIPULATE_POWER_COVER, ManipulatePowerCover)
            print "Test 20, 0.5" 
            manipulatePowerCover(deg2rad(20), 0.5)
            wait()
            assert last_state == 'DOWN'
            rospy.loginfo("Case 1 Done.")

            print "Test 40, 0.75"
            manipulatePowerCover(deg2rad(40), 0.75)
            wait()
            assert last_state == 'DOWN'
            rospy.loginfo("Case 2 Done.")

            print "Test 50, 1"
            manipulatePowerCover(deg2rad(50), 1)
            wait()
            assert last_state == 'UP'
            rospy.loginfo("Case 3 Done.")

            print "Test -25, 0.5"
            manipulatePowerCover(deg2rad(-25), 0.5)
            wait()
            assert last_state == 'UP'
            rospy.loginfo("Case 4 Done.")

            print "Test -40, 0.5"
            manipulatePowerCover(deg2rad(-40), 0.5)
            wait()
            assert last_state == 'UP'
            rospy.loginfo("Case 5 Done.")

            print "Test -50, 0.5"
            manipulatePowerCover(deg2rad(-50), 0.5)
            wait()
            assert last_state == 'DOWN'
            rospy.loginfo("Case 6 Done.")

            rospy.loginfo("Test 70, 0.75")
            manipulatePowerCover(deg2rad(70), 0.75)
            wait()
            assert last_state == 'UP'
            rospy.loginfo("Case 7 Done.")

            print "Test -80, 1"
            manipulatePowerCover(deg2rad(-80), 1.0)
            wait()
            assert last_state == 'DOWN'
            rospy.loginfo("Case 8 Done.")

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        time.sleep(2.0)

if __name__ == '__main__':
    print "Waiting for test to start at time "
    rostest.run(PKG, NAME, PowerCoverTest, sys.argv)
