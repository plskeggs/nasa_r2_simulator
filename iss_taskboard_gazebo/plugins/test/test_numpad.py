#!/usr/bin/env python
##
# Copyright (C) 2013 TopCoder Inc., All Rights Reserved.
#
# @author KennyAlive
# @version 1.0
#

## Tests for numpad

PKG = 'iss_taskboard_gazebo'
NAME = 'test_numpad'

import math
import roslib
roslib.load_manifest(PKG)

import sys, unittest
import os, os.path, threading, time
import rospy, rostest

from std_msgs.msg import String
from helper import *
from iss_taskboard_gazebo.msg import *
from iss_taskboard_gazebo.srv import *

button_states = ['UNSEL', 'UNSEL', 'UNSEL', 'UNSEL', 'UNSEL', 'UNSEL', 'UNSEL', 'UNSEL', 'UNSEL']
led_states = ['OFF', 'OFF', 'OFF', 'OFF', 'OFF', 'OFF', 'OFF', 'OFF', 'OFF']

def onMessage(message):
    global button_states
    if button_states[0] != message.A02_NUM_PAD_A1:
        rospy.loginfo("Num Pad button A1: " + message.A02_NUM_PAD_A1)
    button_states[0] = message.A02_NUM_PAD_A1
    
    if button_states[1] != message.A02_NUM_PAD_A2:
        rospy.loginfo("Num Pad button A2: " + message.A02_NUM_PAD_A2)
    button_states[1] = message.A02_NUM_PAD_A2
    
    if button_states[2] != message.A02_NUM_PAD_A3:
        rospy.loginfo("Num Pad button A3: " + message.A02_NUM_PAD_A3)
    button_states[2] = message.A02_NUM_PAD_A3
    
    if button_states[3] != message.A02_NUM_PAD_B1:
        rospy.loginfo("Num Pad button B1: " + message.A02_NUM_PAD_B1)
    button_states[3] = message.A02_NUM_PAD_B1
    
    if button_states[4] != message.A02_NUM_PAD_B2:
        rospy.loginfo("Num Pad button B2: " + message.A02_NUM_PAD_B2)
    button_states[4] = message.A02_NUM_PAD_B2
    
    if button_states[5] != message.A02_NUM_PAD_B3:
        rospy.loginfo("Num Pad button B3: " + message.A02_NUM_PAD_B3)
    button_states[5] = message.A02_NUM_PAD_B3
    
    if button_states[6] != message.A02_NUM_PAD_C1:
        rospy.loginfo("Num Pad button C1: " + message.A02_NUM_PAD_C1)
    button_states[6] = message.A02_NUM_PAD_C1
    
    if button_states[7] != message.A02_NUM_PAD_C2:
        rospy.loginfo("Num Pad button C2: " + message.A02_NUM_PAD_C2)
    button_states[7] = message.A02_NUM_PAD_C2
    
    if button_states[8] != message.A02_NUM_PAD_C3:
        rospy.loginfo("Num Pad button C3: " + message.A02_NUM_PAD_C3)
    button_states[8] = message.A02_NUM_PAD_C3

    global led_states
    if led_states[0] != message.A02_LED_NUM_PAD_A1:
        rospy.loginfo("Num Pad led A1: " + message.A02_LED_NUM_PAD_A1)
    led_states[0] = message.A02_LED_NUM_PAD_A1

    if led_states[1] != message.A02_LED_NUM_PAD_A2:
        rospy.loginfo("Num Pad led A2: " + message.A02_LED_NUM_PAD_A2)
    led_states[1] = message.A02_LED_NUM_PAD_A2

    if led_states[2] != message.A02_LED_NUM_PAD_A3:
        rospy.loginfo("Num Pad led A3: " + message.A02_LED_NUM_PAD_A3)
    led_states[2] = message.A02_LED_NUM_PAD_A3

    if led_states[3] != message.A02_LED_NUM_PAD_B1:
        rospy.loginfo("Num Pad led B1: " + message.A02_LED_NUM_PAD_B1)
    led_states[3] = message.A02_LED_NUM_PAD_B1

    if led_states[4] != message.A02_LED_NUM_PAD_B2:
        rospy.loginfo("Num Pad led B2: " + message.A02_LED_NUM_PAD_B2)
    led_states[4] = message.A02_LED_NUM_PAD_B2

    if led_states[5] != message.A02_LED_NUM_PAD_B3:
        rospy.loginfo("Num Pad led B3: " + message.A02_LED_NUM_PAD_B3)
    led_states[5] = message.A02_LED_NUM_PAD_B3

    if led_states[6] != message.A02_LED_NUM_PAD_C1:
        rospy.loginfo("Num Pad led C1: " + message.A02_LED_NUM_PAD_C1)
    led_states[6] = message.A02_LED_NUM_PAD_C1

    if led_states[7] != message.A02_LED_NUM_PAD_C2:
        rospy.loginfo("Num Pad led C2: " + message.A02_LED_NUM_PAD_C2)
    led_states[7] = message.A02_LED_NUM_PAD_C2

    if led_states[8] != message.A02_LED_NUM_PAD_C3:
        rospy.loginfo("Num Pad led C3: " + message.A02_LED_NUM_PAD_C3)
    led_states[8] = message.A02_LED_NUM_PAD_C3

class NumPadTest(unittest.TestCase):
    def __init__(self, *args):
       super(NumPadTest, self).__init__(*args)

    def test_numpad(self):
        rospy.init_node(NAME, anonymous=True)

        rospy.Subscriber(TOPIC_NAME, TaskboardPanelA, onMessage)

        rospy.loginfo("Waiting for gazebo ...")
        rospy.wait_for_service(SERVICE_MANIPULATE_POWER_COVER)
        rospy.wait_for_service(SERVICE_MANIPULATE_POWER_SWITCH)
        rospy.wait_for_service(SERVICE_MANIPULATE_NUMPAD)
        wait(GUI_WAIT_TIME)
        rospy.loginfo("Testing has started")

        try:
            manipulatePowerCover = rospy.ServiceProxy(SERVICE_MANIPULATE_POWER_COVER, ManipulatePowerCover)
            manipulatePowerSwitch = rospy.ServiceProxy(SERVICE_MANIPULATE_POWER_SWITCH, ManipulatePowerSwitch)
            manipulateNumPad = rospy.ServiceProxy(SERVICE_MANIPULATE_NUMPAD, ManipulateNumPad)

            # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            # Open power cover
            manipulatePowerCover(deg2rad(50), 0.5)
            wait()
            # Turn on power
            manipulatePowerSwitch(deg2rad(30), 0.5)
            wait()
            ## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            # Turn on all leds
            for i in range(9):
                manipulateNumPad(i, 20.0, 0.25)
                wait(1.5)
                assert led_states[i] == 'ON'
                rospy.loginfo("Case Done.")

            # Turn off some leds
            for i in range(1, 9, 2):
                manipulateNumPad(i, 20.0, 0.5)
                wait()
                assert led_states[i-1] == 'ON'
                assert led_states[i] == 'OFF'
                rospy.loginfo("Case Done")

            ## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            # Turn off the power
            manipulatePowerSwitch(deg2rad(-30), 0.5)
            wait()
            # Close power cover
            manipulatePowerCover(deg2rad(-50), 0.5)
            wait()
            ## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            # Check that all leds are off
            for i in range(9):
                assert led_states[i] == 'OFF'
            rospy.loginfo("Case Done")
            
            # Without power led is off
            manipulateNumPad(0, 20.0, 1.0)
            wait()
            assert led_states[0] == 'OFF'
            rospy.loginfo("Case Done")

            # Press on button long enough and check that it is in SEL state
            manipulateNumPad(3, 20.0, 8.0)
            wait(1)
            assert button_states[3] == 'SEL'
            rospy.loginfo("Case Done")

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        time.sleep(0.5)

if __name__ == '__main__':
    print "Waiting for test to start at time "
    rostest.run(PKG, 'test_numpad', NumPadTest)
