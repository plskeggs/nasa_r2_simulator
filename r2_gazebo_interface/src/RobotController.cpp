#include "r2_gazebo_interface/RobotController.h"

using namespace gazebo;

RobotController::RobotController(physics::ModelPtr _modelPtr)
    : modelPtr(_modelPtr)
{
    prevUpdateTime = _modelPtr->GetWorld()->GetSimTime();
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
    if(_parentName != _childName)
    {
        joints[_childName]->setPassive();
    }
}

void RobotController::setPosPid(const std::string& name, double _p, double _i, double _d,
                                double _imax, double _imin, double _cmdMax, double _cmdMin)
{
    if (joints.find(name) == joints.end())
    {
        ROS_DEBUG("GazeboInterface PosPID not set because joint (%s) not found", name.c_str());
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

    if (!is_paused)
    {
        modelPtr->GetWorld()->SetPaused(true);
    }

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

        if (!posMap.empty())
        {
            getJointStates(posMap);
        }
    }

    physics::Joint_V model_joints = modelPtr->GetJoints();


    for (physics::Joint_V::iterator j_it = model_joints.begin(); j_it != model_joints.end(); ++j_it)
    {
        // joint
        std::string name = (*j_it)->GetName();
        std::map<std::string, JointState>::iterator it = posMap.find(name);

        if (it != posMap.end())
        {
            it->second.position = (*j_it)->GetAngle(0).Radian();
            it->second.velocity = (*j_it)->GetVelocity(0);
            it->second.effort = (*j_it)->GetForce((unsigned int)0);
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

        // add all controllable joints
        for (std::map<std::string, JointControllerPtr>::const_iterator it = joints.begin(); it != joints.end(); ++it)
        {
            if (!it->second->isPassive())
            {
                posMap[it->first] = zero;
            }
        }

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

            //check if joint is self-dependent
            dependenciesType::iterator depIt = dependencies.find(jIt->first);
            if(depIt != dependencies.end())
            {
                it->second.position = it->second.position / depIt->second.second;
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
            if(!it->second->isPassive())
            {
                limitsMap[it->first] = std::pair<double, double>(-3.14159, 3.14159);
            }
        }

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
    if(!joints[name]->isPassive())
    {
        joints[name]->setPosTarget(target);
    }

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
    if(!joints[name]->isPassive())
    {
        joints[name]->setVelTarget(target);
    }

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
    if(!joints[name]->isPassive())
    {
        joints[name]->setEffortTarget(target);
    }

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
    common::Time currTime = modelPtr->GetWorld()->GetSimTime();
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
