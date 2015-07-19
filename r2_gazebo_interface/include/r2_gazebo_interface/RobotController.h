#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>
#include <r2_gazebo_interface/JointController.h>
#include <nasa_r2_common_msgs/JointStatusArray.h>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <boost/thread.hpp>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace gazebo
{
    class RobotController
    {
    public:
        struct JointState
        {
            double position;
            double velocity;
            double effort;
        };

        RobotController(physics::ModelPtr _modelPtr);
        ~RobotController();

        // add/modify joints
        void addJoint(physics::JointPtr _jointPtr, bool advancedMode = false);
        // child pos/vel/effort scaled to factor * parent
        // if child and parent are the same joint, this effectively scales the input for that joint
        void addJointDependency(std::string _childName, std::string _parentName, double factor);
        void setPosPid(const std::string& name, double _p = 0.0, double _i = 0.0, double _d = 0.0,
                       double _imax = 0.0, double _imin = 0.0,
                       double _cmdMax = 0.0, double _cmdMin = 0.0);
        void setVelPid(const std::string& name, double _p = 0.0, double _i = 0.0, double _d = 0.0,
                       double _imax = 0.0, double _imin = 0.0,
                       double _cmdMax = 0.0, double _cmdMin = 0.0);

        // move the robot to the specified positions while taking dependencies into account
        void setJointPositions(std::map<std::string, double> posMap);
        // get states
        // if map is empty, get all, ignoring dependent joints
        void getJointStates(std::map<std::string, JointState>& posMap);
        // get joint targets
        // if map is empty, get all, ignoring dependent joints
        void getJointTargets(std::map<std::string, JointState>& posMap);

        // get joint limits
        // if map is empty, get all, ignoring dependent joints
        void getJointLimits(std::map<std::string, std::pair<double, double> >& limitsMap);

        // set targets
        void setJointPosTarget(const std::string& name, double target);
        void setJointVelTarget(const std::string& name, double target);
        void setJointEffortTarget(const std::string& name, double target);

        // joint state management
        void setJointControl(const nasa_r2_common_msgs::JointControl::ConstPtr& msg);
        const nasa_r2_common_msgs::JointStatus& getJointStatus(const std::string& name) const;
        void publishJointStatuses(ros::Publisher& rosPub) const;

        // update PIDs and send forces to joints
        void update();

    private:
        physics::ModelPtr modelPtr;
        std::map<std::string, JointControllerPtr> joints;

        typedef std::multimap<std::string, std::pair<std::string, double> > dependenciesType;
        dependenciesType dependencies;

        common::Time prevUpdateTime;
    };
}
#endif
