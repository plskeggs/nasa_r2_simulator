#!/usr/bin/env python
##
# Copyright (C) 2013 TopCoder Inc., All Rights Reserved.
#
# @author KennyAlive
# @version 1.0
#

import time, math

# Time the test wait for gui loading
GUI_WAIT_TIME = 25.0

# Default pause time for short actions
DEFAULT_PAUSE = 25.0

# The topic's parent name
TOPIC_NAME = '/taskboard/TaskboardPanelA'

# The service's parent name
SERVICE_PARENT = '/taskboard'

# Available services
SERVICE_MANIPULATE_POWER_COVER   = SERVICE_PARENT + '/manipulate_power_cover'
SERVICE_MANIPULATE_POWER_SWITCH  = SERVICE_PARENT + '/manipulate_power_switch'
SERVICE_MANIPULATE_ROCKER_SWITCH = SERVICE_PARENT + '/manipulate_rocker_switch_a01'
SERVICE_MANIPULATE_NUMPAD        = SERVICE_PARENT + '/manipulate_numpad'
SERVICE_MANIPULATE_SAFE_TOGGLE   = SERVICE_PARENT + '/manipulate_safe_toggle'

def deg2rad(degrees):
    return math.pi * degrees / 180

def wait(pause = DEFAULT_PAUSE):
    time.sleep(pause)
