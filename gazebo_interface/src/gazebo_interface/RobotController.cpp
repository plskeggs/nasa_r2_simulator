#include <gazebo_interface/RobotController.h>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>

#include <ros/ros.h>

#include <nasa_r2_common_msgs/JointStatusArray.h>

using namespace gazebo;

RobotController::RobotController(physics::ModelPtr _modelPtr)
: modelPtr(_modelPtr)
{
    prevUpdateTime = _modelPtr->GetWorld()->SimTime();
}

RobotController::~RobotController()
{
}

void RobotController::addJoint(physics::JointPtr _jointPtr, bool advancedMode)
{
    boost::mutex::scoped_lock(pidMutex);
    JointControllerPtr jointPtr(new JointController(_jointPtr, advancedMode));
    joints[_jointPtr->GetName()] = jointPtr;
}

void RobotController::addJointDependency(std::string _childName, std::string _parentName, double factor)
{
    if (joints.find(_parentName) == joints.end())
    {
        ROS_WARN("GazeboInterface joint dependency not set because parent (%s) not found", _parentName.c_str());
        return;
    }
    if (joints.find(_childName) == joints.end())
    {
        ROS_WARN("GazeboInterface joint dependency not set because child (%s) not found", _childName.c_str());
        return;
    }
    boost::mutex::scoped_lock(pidMutex);
    dependencies.insert(std::make_pair(_parentName, std::make_pair(_childName, factor)));
}

void RobotController::setPosPid(const std::string& name, double _p, double _i, double _d,
                                  double _imax, double _imin, double _cmdMax, double _cmdMin)
{
    if (joints.find(name) == joints.end())
    {
        ROS_WARN("GazeboInterface PosPID not set because joint (%s) not found", name.c_str());
        return;
    }
    boost::mutex::scoped_lock(pidMutex);
    joints[name]->setPosPid(_p, _i, _d, _imax, _imin, _cmdMax, _cmdMin);
}

void RobotController::setVelPid(const std::string& name, double _p, double _i, double _d,
                                  double _imax, double _imin, double _cmdMax, double _cmdMin)
{
    if (joints.find(name) == joints.end())
    {
        ROS_WARN("GazeboInterface PosPID not set because joint (%s) not found", name.c_str());
        return;
    }
    boost::mutex::scoped_lock(pidMutex);
    joints[name]->setVelPid(_p, _i, _d, _imax, _imin, _cmdMax, _cmdMin);
}

// move the robot to the specified positions while taking dependencies into account
void RobotController::setJointPositions(std::map<std::string, double> posMap)
{
    // pause gazebo
    bool is_paused = modelPtr->GetWorld()->IsPaused();
    if (!is_paused) modelPtr->GetWorld()->SetPaused(true);

    // release brakes on all joints
    std::map<std::string, bool> brakeStates;
    for (std::map<std::string, JointControllerPtr>::iterator it = joints.begin(); it != joints.end(); ++it)
    {
        brakeStates[it->first] = it->second->getJointStatus().brakeReleased;
        it->second->releaseBrake();
    }

    // check for dependencies
    for (std::map<std::string, double>::iterator posIt = posMap.begin(); posIt != posMap.end(); ++posIt)
    {
        std::pair<dependenciesType::iterator, dependenciesType::iterator> deps = dependencies.equal_range(posIt->first);
        for (dependenciesType::iterator depIt = deps.first; depIt != deps.second; ++depIt)
        {
            posMap[depIt->second.first] = depIt->second.second * posIt->second;
        }
    }

    // set positions
    modelPtr->SetJointPositions(posMap);

    // reset brakes on all joints
    for (std::map<std::string, bool>::iterator it = brakeStates.begin(); it != brakeStates.end(); ++it)
    {
        joints[it->first]->releaseBrake(it->second);
    }

    // resume paused state
    modelPtr->GetWorld()->SetPaused(is_paused);
}

void RobotController::getJointStates(std::map<std::string, JointState>& posMap)
{
    if (posMap.empty())
    {
        JointState zero;
        zero.position = 0.;
        zero.velocity = 0.;
        zero.effort = 0.;

        // add all joints
        for (std::map<std::string, JointControllerPtr>::const_iterator it = joints.begin(); it != joints.end(); ++it)
        {
            posMap[it->first] = zero;
        }

        // remove dependencies
//        for (dependenciesType::const_iterator depIt = dependencies.begin(); depIt != dependencies.end(); ++depIt)
//        {
//            if (depIt->first != depIt->second.first)
//            {
//                posMap.erase(depIt->second.first);
//            }
//        }

        if (!posMap.empty())
        {
            getJointStates(posMap);
        }
    }

    std::vector<physics::JointPtr> joints = modelPtr->GetJoints();
    for (unsigned int i = 0; i < joints.size(); ++i)
    {
        physics::JointPtr jPtr = joints[i];

        // joint
        std::string name = jPtr->GetName();
        std::map<std::string, JointState>::iterator it = posMap.find(name);
        if (it != posMap.end())
        {
            it->second.position = jPtr->Position(0);
            it->second.velocity = jPtr->GetVelocity(0);
            it->second.effort = jPtr->GetForce(0);
        }
    }
}

void RobotController::getJointTargets(std::map<std::string, JointState>& posMap)
{
    if (posMap.empty())
    {
        JointState zero;
        zero.position = 0.;
        zero.velocity = 0.;
        zero.effort = 0.;

        // add all joints
        for (std::map<std::string, JointControllerPtr>::const_iterator it = joints.begin(); it != joints.end(); ++it)
        {
            posMap[it->first] = zero;
        }

        // remove dependencies
//        for (dependenciesType::const_iterator depIt = dependencies.begin(); depIt != dependencies.end(); ++depIt)
//        {
//            if (depIt->first != depIt->second.first)
//            {
//                posMap.erase(depIt->second.first);
//            }
//        }

        if (!posMap.empty())
        {
            getJointTargets(posMap);
        }
    }

    for (std::map<std::string, JointControllerPtr>::const_iterator jIt = joints.begin(); jIt != joints.end(); ++jIt)
    {
        std::map<std::string, JointState>::iterator it = posMap.find(jIt->first);
        if (it != posMap.end())
        {
            it->second.position = jIt->second->getPosTarget();
            it->second.velocity = jIt->second->getVelTarget();
            it->second.effort = jIt->second->getEffortTarget();

            // check for dependencies
            std::pair<dependenciesType::iterator, dependenciesType::iterator> deps = dependencies.equal_range(jIt->first);
            for (dependenciesType::iterator depIt = deps.first; depIt != deps.second; ++depIt)
            {
                if(depIt->second.second!=0)
                {
                    it->second.position=it->second.position/depIt->second.second;

                }
            }
        }
    }
}

void RobotController::getJointLimits(std::map<std::string, std::pair<double, double> >& limitsMap)
{
    if (limitsMap.empty())
    {
        // add all joints
        for (std::map<std::string, JointControllerPtr>::const_iterator it = joints.begin(); it != joints.end(); ++it)
        {
            limitsMap[it->first] = std::pair<double, double>(-3.14159, 3.14159);
        }

        // remove dependencies
//        for (dependenciesType::const_iterator depIt = dependencies.begin(); depIt != dependencies.end(); ++depIt)
//        {
//            if (depIt->first != depIt->second.first)
//            {
//                limitsMap.erase(depIt->second.first);
//            }
//        }

        if (!limitsMap.empty())
        {
            getJointLimits(limitsMap);
        }
    }

    for (std::map<std::string, JointControllerPtr>::const_iterator jIt = joints.begin(); jIt != joints.end(); ++jIt)
    {
        std::map<std::string, std::pair<double, double> >::iterator it = limitsMap.find(jIt->first);
        if (it != limitsMap.end())
        {
            jIt->second->getJointLimits(it->second.first, it->second.second);
        }
    }
}

void RobotController::setJointPosTarget(const std::string& name, double target)
{
    if (joints.find(name) == joints.end())
    {
        //ROS_WARN("GazeboInterface setJointPosTarget not set because joint (%s) not found", name.c_str());
        return;
    }
    boost::mutex::scoped_lock(pidMutex);
    joints[name]->setPosTarget(target);

    // check for dependencies
    std::pair<dependenciesType::iterator, dependenciesType::iterator> deps = dependencies.equal_range(name);
    for (dependenciesType::iterator depIt = deps.first; depIt != deps.second; ++depIt)
    {
        joints[depIt->second.first]->setPosTarget(depIt->second.second * target);
    }
}

void RobotController::setJointVelTarget(const std::string& name, double target)
{
    if (joints.find(name) == joints.end())
    {
        ROS_WARN("GazeboInterface setJointVelTarget not set because joint (%s) not found", name.c_str());
        return;
    }
    boost::mutex::scoped_lock(pidMutex);
    joints[name]->setVelTarget(target);

    // check for dependencies
    std::pair<dependenciesType::iterator, dependenciesType::iterator> deps = dependencies.equal_range(name);
    for (dependenciesType::iterator depIt = deps.first; depIt != deps.second; ++depIt)
    {
        joints[depIt->second.first]->setVelTarget(depIt->second.second * target);
    }
}

void RobotController::setJointEffortTarget(const std::string& name, double target)
{
    if (joints.find(name) == joints.end())
    {
        ROS_WARN("GazeboInterface setJointEffortTarget not set because joint (%s) not found", name.c_str());
        return;
    }
    boost::mutex::scoped_lock(pidMutex);
    joints[name]->setEffortTarget(target);

    // check for dependencies
    std::pair<dependenciesType::iterator, dependenciesType::iterator> deps = dependencies.equal_range(name);
    for (dependenciesType::iterator depIt = deps.first; depIt != deps.second; ++depIt)
    {
        joints[depIt->second.first]->setEffortTarget(depIt->second.second * target);
    }
}

// joint state management
void RobotController::setJointControl(const nasa_r2_common_msgs::JointControl::ConstPtr& msg)
{
    if (joints.find(msg->joint) == joints.end())
    {
        ROS_WARN("GazeboInterface setJointControl not set because joint (%s) not found", msg->joint.c_str());
        return;
    }
    boost::mutex::scoped_lock(pidMutex);
    joints[msg->joint]->setJointControl(msg);

    // check for dependencies
    std::pair<dependenciesType::iterator, dependenciesType::iterator> deps = dependencies.equal_range(msg->joint);
    for (dependenciesType::iterator depIt = deps.first; depIt != deps.second; ++depIt)
    {
        nasa_r2_common_msgs::JointControl* jcPtr = new nasa_r2_common_msgs::JointControl(*msg);
        jcPtr->joint = depIt->second.first;
        nasa_r2_common_msgs::JointControl::ConstPtr depJointControlPtr(jcPtr);
        joints[depIt->second.first]->setJointControl(depJointControlPtr);
    }
}

const nasa_r2_common_msgs::JointStatus& RobotController::getJointStatus(const std::string& name) const
{
    std::map<std::string, JointControllerPtr>::const_iterator cit = joints.find(name);
    if (cit == joints.end())
    {
        ROS_WARN("GazeboInterface getJointStatus failed because joint (%s) not found", name.c_str());
    }
    return cit->second->getJointStatus();
}

void RobotController::update()
{
    common::Time currTime = modelPtr->GetWorld()->SimTime();
    common::Time stepTime = currTime - prevUpdateTime;
    prevUpdateTime = currTime;

    for (std::map<std::string, JointControllerPtr>::iterator it = joints.begin(); it != joints.end(); ++it)
    {
        it->second->update(stepTime);
    }
}

void RobotController::publishJointStatuses(ros::Publisher& rosPub) const
{
    nasa_r2_common_msgs::JointStatusArray statusArray;
    statusArray.header.stamp = ros::Time::now();
    for (std::map<std::string, JointControllerPtr>::const_iterator it = joints.begin(); it != joints.end(); ++it)
    {
        statusArray.status.push_back(it->second->getJointStatus());
    }
    rosPub.publish(statusArray);
}
