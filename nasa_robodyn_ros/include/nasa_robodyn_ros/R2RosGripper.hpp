/**************************************************************************
** Copyright (c) 2015 United States Government as represented by the
** National Aeronotics and Space Administration.  All Rights Reserved
**
** Author: Allison Thackston
** Created: 20 Feb 2015
**
** Developed jointly by NASA/JSC and Oceaneering Space Systems
**
** Licensed under the NASA Open Source Agreement v1.3 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://opensource.org/licenses/NASA-1.3
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
**************************************************************************/

#ifndef R2ROSGRIPPER_H
#define R2ROSGRIPPER_H

#include <ros/ros.h>
#include <nasa_r2_common_msgs/GripperPositionCommand.h>
#include <nasa_r2_common_msgs/JointTrajectoryReplan.h>
#include <nasa_r2_common_msgs/ControllerJointSettings.h>

class R2RosGripper
{
public:
    R2RosGripper(ros::NodeHandle &nh);

    void configure(std::vector<std::string> names, double velocity, double acceleration);
    void gripperCmdCallback(const nasa_r2_common_msgs::GripperPositionCommand& msg);

private:
    ros::NodeHandle nh;
    nasa_r2_common_msgs::GripperPositionCommand gripperCmd;

    ros::Subscriber gripperCmdIn;

    ros::Publisher gripperCmdOut;
    ros::Publisher gripperSettingsOut;

    double velocity;
    double acceleration;
};

#endif // R2ROSGRIPPER_H
