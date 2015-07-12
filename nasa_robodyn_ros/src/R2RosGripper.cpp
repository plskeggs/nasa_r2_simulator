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

#include "nasa_robodyn_ros/R2RosGripper.hpp"

R2RosGripper::R2RosGripper(ros::NodeHandle& nh)
{
    this->nh = nh;

    gripperCmdIn = nh.subscribe("gripper_cmd", 10, &R2RosGripper::gripperCmdCallback, this);

    gripperCmdOut = nh.advertise<nasa_r2_common_msgs::JointTrajectoryReplan>("joint_refs", 1);
    gripperSettingsOut = nh.advertise<nasa_r2_common_msgs::ControllerJointSettings>("joint_ref_settings", 1, true);
}

void R2RosGripper::configure(std::vector<std::string> names, double velocity, double acceleration)
{
    nasa_r2_common_msgs::ControllerJointSettings settings;
    settings.joint_names = names;
    settings.jointVelocityLimits.resize(names.size());
    settings.jointAccelerationLimits.resize(names.size());
    for(unsigned int i = 0; i < names.size(); ++i)
    {
        settings.jointVelocityLimits[i] = velocity;
        settings.jointAccelerationLimits[i] = acceleration;
    }
    gripperSettingsOut.publish(settings);
}

void R2RosGripper::gripperCmdCallback(const nasa_r2_common_msgs::GripperPositionCommand &msg)
{
    ROS_INFO("in gripper_cmd callback");
    nasa_r2_common_msgs::JointTrajectoryReplan gripperCmd;
    trajectory_msgs::JointTrajectory gripperTraj;
    gripperTraj.joint_names = msg.name;
    gripperTraj.header = msg.header;

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(msg.name.size());
    point.velocities.resize(msg.name.size());
    point.accelerations.resize(msg.name.size());
    point.time_from_start = ros::Duration(0.1);

    for(unsigned int i = 0; i < msg.name.size(); ++i)
    {
        ROS_INFO("name: %s", msg.name[i].c_str());
        ROS_INFO("command: %s", msg.command[i].c_str());
        ROS_INFO("setpoint: %s", msg.setpoint[i].c_str());

        if(msg.command[i] == "set")
        {
            if(msg.setpoint[i] == "closed")
            {
                ROS_INFO("setting position closed");
                point.positions[i] = 0;
            }
            else if (msg.setpoint[i] == "seattrack")
            {
                ROS_INFO("setting position seattrack");
                point.positions[i] = 0.15;
            }

        }
        else if(msg.command[i] == "lock")
        {
            ROS_INFO("setting lock");
            point.positions[i] = -0.1;
        }
        else if(msg.command[i] == "release")
        {
            ROS_INFO("setting release");
            point.positions[i] = 0.5;
        }
        else
        {
            return;
        }
    }
    gripperTraj.points.push_back(point);
    gripperTraj.header.frame_id="gripper_cmd";
    gripperCmd.trajectory = gripperTraj;
    gripperCmd.header = gripperTraj.header;
    gripperCmdOut.publish(gripperCmd);
}
