#include <gazebo_interface/GazeboInterface.h>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>
#include <nasa_r2_common_msgs/JointStatusArray.h>

#include <ros/console.h>

using namespace gazebo;

GazeboInterface::GazeboInterface()
: ModelPlugin()
, advancedMode(false)
{
}

GazeboInterface::~GazeboInterface()
{
    // shutdown ros
    rosNodePtr->shutdown();
}

void GazeboInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    modelPtr = _model;

    prevStatesUpdateTime = _model->GetWorld()->SimTime();
    prevStatusUpdateTime = _model->GetWorld()->SimTime();

    // create controller
    robotControllerPtr.reset(new RobotController(modelPtr));

    // load parameters
    std::string robotNamespace = "";
    if (_sdf->HasElement("robotNamespace"))
    {
        robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    std::string paramsNamespace = "";
    if (_sdf->HasElement("paramsNamespace"))
    {
        paramsNamespace = _sdf->GetElement("paramsNamespace")->Get<std::string>() + "/";
    }

    if (!_sdf->HasElement("jointCommandsTopic"))
    {
        ROS_FATAL("GazeboInterface plugin missing <jointCommandsTopic>, cannot proceed");
        return;
    }
    else
        jointCommandsTopic = _sdf->GetElement("jointCommandsTopic")->Get<std::string>();

    if (!_sdf->HasElement("jointStatesTopic"))
    {
        ROS_FATAL("GazeboInterface plugin missing <jointStatesTopic>, cannot proceed");
        return;
    }
    else
        jointStatesTopic = _sdf->GetElement("jointStatesTopic")->Get<std::string>();

    if (!_sdf->HasElement("jointCapabilitiesTopic"))
    {
        ROS_FATAL("GazeboInterface plugin missing <jointCapabilitiesTopic>, cannot proceed");
        return;
    }
    else
        jointCapabilitiesTopic = _sdf->GetElement("jointCapabilitiesTopic")->Get<std::string>();

    if (!_sdf->HasElement("jointStatesRate"))
    {
        jointStatesStepTime = 0.;
    }
    else
        jointStatesStepTime = 1./_sdf->GetElement("jointStatesRate")->Get<double>();

    if (!_sdf->HasElement("jointCommandRefsTopic"))
    {
        ROS_FATAL("GazeboInterface plugin missing <jointCommandRefsTopic>, cannot proceed");
        return;
    }
    else
        jointCommandRefsTopic = _sdf->GetElement("jointCommandRefsTopic")->Get<std::string>();

    if (!_sdf->HasElement("advancedMode"))
    {
        ROS_INFO("GazeboInterface plugin missing <advancedMode>, defaults to false");
        advancedMode = false;
    }
    else
    {
        std::string strVal = _sdf->GetElement("advancedMode")->Get<std::string>();
        if (strVal == "1")
            advancedMode = true;
        else if (strVal == "0")
            advancedMode = false;
        else
        {
            ROS_WARN("GazeboInterface plugin <advancedMode> should be type bool (true/false), defaults to false");
            advancedMode = false;
        }
    }

    ROS_INFO("GazeboInterface plugin advancedMode: %s", advancedMode ? "true" : "false");

    if (advancedMode)
    {
        if (!_sdf->HasElement("jointControlTopic"))
        {
            ROS_FATAL("GazeboInterface plugin missing <jointControlTopic>, cannot proceed");
            return;
        }
        else
            jointControlTopic = _sdf->GetElement("jointControlTopic")->Get<std::string>();

        if (!_sdf->HasElement("jointStatusTopic"))
        {
            ROS_FATAL("GazeboInterface plugin missing <jointStatusTopic>, cannot proceed");
            return;
        }
        else
            jointStatusTopic = _sdf->GetElement("jointStatusTopic")->Get<std::string>();

        if (!_sdf->HasElement("jointStatusRate"))
        {
            jointStatusStepTime = 0.;
        }
        else
            jointStatusStepTime = 1./_sdf->GetElement("jointStatusRate")->Get<double>();
    }

    // create ros nodes
    rosNodePtr.reset(new ros::NodeHandle(robotNamespace));
    paramsNodePtr.reset(new ros::NodeHandle(paramsNamespace));

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnectionPtr = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboInterface::update, this));

    ROS_INFO("Gazebo Interface plugin loaded");
}

void GazeboInterface::traverseParams(XmlRpc::XmlRpcValue param, std::map<std::string, XmlRpc::XmlRpcValue>& valMap, std::string searchKey,
                    std::string ns, std::string name)
{
    std::string fullName;
    if (name.empty())
    {
        fullName = ns;
    }
    else
    {
        fullName = ns + "/" + name;
    }

    if (param.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
        // if it is a struct, recurse
        for (XmlRpc::XmlRpcValue::iterator paramIt = param.begin(); paramIt != param.end(); ++paramIt)
        {
            traverseParams(paramIt->second, valMap, searchKey, fullName, paramIt->first);
        }
    }
    else if (searchKey == "")
    {
        // if there is no search key, add to map with fully qualified name
        valMap.insert(std::make_pair(fullName, param));
    }
    else if (searchKey == name)
    {
        // if the searchKey matches the name, add to map omitting searchKey
        valMap.insert(std::make_pair(ns, param));
    }

    // if searchKey is not matched, the function does nothing
}

bool GazeboInterface::getDoubleVal(XmlRpc::XmlRpcValue& val, double& doubleVal)
{
    if (val.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        doubleVal = (double)(int)val;
    }
    else if (val.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        doubleVal = (double)val;
    }
    else
    {
        return false;
    }

    return true;
}

void GazeboInterface::Init()
{
    ROS_DEBUG("add joints");
    // add all joints to controller
    physics::Joint_V _joints = modelPtr->GetJoints();
    for (unsigned int i = 0; i < modelPtr->GetJointCount(); ++i)
    {
        physics::JointPtr jPtr = _joints[i];
        robotControllerPtr->addJoint(jPtr, advancedMode);
        ROS_DEBUG("Add %s to controller", jPtr->GetName().c_str());
    }

    XmlRpc::XmlRpcValue param;
    // set joint dependencies
    if (paramsNodePtr->getParam("dependency", param))
    {
        ROS_DEBUG("set dependencies");
        // get dependent children
        std::map<std::string, XmlRpc::XmlRpcValue> dependentChildrenMap;
        traverseParams(param, dependentChildrenMap, "children");
        // get dependent factors
        std::map<std::string, XmlRpc::XmlRpcValue> dependentFactorsMap;
        traverseParams(param, dependentFactorsMap, "factors");

        // iterate through and add dependencies
        for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator depIt = dependentChildrenMap.begin();
             depIt != dependentChildrenMap.end(); ++depIt)
        {
            XmlRpc::XmlRpcValue factorVal = dependentFactorsMap[depIt->first];
            // value can be a string or an array of strings
            if (depIt->second.getType() == XmlRpc::XmlRpcValue::TypeString)
            {
                double val;
                if (getDoubleVal(factorVal, val))
                {
                    // child, parent, factor
                    ROS_DEBUG("Add joint dependency: %s, %s, %f", ((std::string)depIt->second).c_str(), depIt->first.c_str(), (double)factorVal);
                    robotControllerPtr->addJointDependency((std::string)depIt->second, depIt->first, (double)factorVal);
                }
                else
                {
                    ROS_WARN("Add dependency for %s (child) to %s (parent) not completed because factor not an appropriate type (double or int)",
                             ((std::string)depIt->second).c_str(), depIt->first.c_str());
                }
            }
            else if (depIt->second.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                if (factorVal.getType() == XmlRpc::XmlRpcValue::TypeArray && factorVal.size() == depIt->second.size())
                {
                    for (int i = 0; i < depIt->second.size(); ++i)
                    {
                        if (depIt->second[i].getType() == XmlRpc::XmlRpcValue::TypeString)
                        {
                            double val;
                            if (getDoubleVal(factorVal[i], val))
                            {
                                // child, parent, factor
                                ROS_DEBUG("Add joint dependency: %s, %s, %f", ((std::string)depIt->second[i]).c_str(), depIt->first.c_str(), val);
                                robotControllerPtr->addJointDependency((std::string)depIt->second[i], depIt->first, val);
                            }
                            else
                            {
                                ROS_WARN("Add dependency for %s (child) to %s (parent) not completed because factor not an appropriate type (double or int)",
                                         ((std::string)depIt->second[i]).c_str(), depIt->first.c_str());
                            }
                        }
                        else
                        {
                            ROS_WARN("A dependency to %s (parent) not added because type is invalid", depIt->first.c_str());
                        }
                    }
                }
                else
                {
                    ROS_WARN("Add dependency to %s (parent) not completed because children/factors array size not the same",
                             depIt->first.c_str());
                }
            }
        }
    }

    // get initial positions
    if (paramsNodePtr->getParam("initial_position", param))
    {
        ROS_DEBUG("set initial positions");
        bool radians = true;
        if (param.hasMember("radians") && param["radians"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
        {
            radians = (bool)param["radians"];
            ROS_DEBUG("radians: %d", radians);
        }
        else
        {
            ROS_WARN("No 'radians' member provided (or invalid type) in 'initial_position'; assumed true");
        }

        std::map<std::string, XmlRpc::XmlRpcValue> initialPoseMapParams;
        traverseParams(param, initialPoseMapParams);
        // remove '/radians'
        initialPoseMapParams.erase("/radians");

        // get positions
        std::map<std::string, double> initialPoseMap;
        for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator poseIt = initialPoseMapParams.begin();
             poseIt != initialPoseMapParams.end(); ++poseIt)
        {
            double val;
            if (getDoubleVal(poseIt->second, val))
            {
                if (radians == false)
                {
                    val = IGN_DTOR(val);
                }

                if (modelPtr->GetJoint(poseIt->first))
                {
                    initialPoseMap.insert(std::make_pair(poseIt->first, val));
                    ROS_DEBUG("initial position of %s set to %f", poseIt->first.c_str(), val);
                }
                else
                {
                    ROS_INFO("initial position of %s ignored because model doesn't contain joint", poseIt->first.c_str());
                }
            }
            else
            {
                ROS_WARN("Sim initial position value (%s) ignored because it is not a valid type (double or int)", poseIt->first.c_str());
            }
        }

        // set positions
        robotControllerPtr->setJointPositions(initialPoseMap);
    }

    // get position PIDsmodelPtr->GetWorld()->SimTime()
    if (paramsNodePtr->getParam("position_pid", param))
    {
        ROS_DEBUG("set position PIDs");
        std::map<std::string, XmlRpc::XmlRpcValue> positionPidMap;
        traverseParams(param, positionPidMap);

        // iterate through and add PIDs
        for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator pidIt = positionPidMap.begin();
             pidIt != positionPidMap.end(); ++pidIt)
        {
            if (pidIt->second.getType() == XmlRpc::XmlRpcValue::TypeArray && pidIt->second.size() >= 3)
            {
                std::vector<double> gains(7, 0.0);
                bool valid = true;
                for (int i = 0; i < pidIt->second.size(); ++ i)
                {
                    valid &= getDoubleVal(pidIt->second[i], gains[i]);
                }

                if (valid)
                {
                    ROS_DEBUG("setPosPid: %s, %f, %f, %f, %f, %f, %f, %f", pidIt->first.c_str(), gains[0], gains[1], gains[2], gains[3], gains[4], gains[5], gains[6]);
                    robotControllerPtr->setPosPid(pidIt->first, gains[0], gains[1], gains[2], gains[3], gains[4], gains[5], gains[6]);
                }
                else
                {
                    ROS_WARN("position PID for %s not set because the values are not valid (double or int)", pidIt->first.c_str());
                }
            }
            else
            {
                ROS_WARN("position PID values for %s not set because it is not an array of size 3 or greater", pidIt->first.c_str());
            }
        }
    }

    // get velocity PIDs
    if (paramsNodePtr->getParam("velocity_pid", param))
    {
        ROS_DEBUG("set velocity PIDs");
        std::map<std::string, XmlRpc::XmlRpcValue> velocityPidMap;
        traverseParams(param, velocityPidMap);

        // iterate through and add PIDs
        for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator pidIt = velocityPidMap.begin();
             pidIt != velocityPidMap.end(); ++pidIt)
        {
            if (pidIt->second.getType() == XmlRpc::XmlRpcValue::TypeArray && pidIt->second.size() >= 3)
            {
                std::vector<double> gains(7, 0.0);
                bool valid = true;
                for (int i = 0; i < pidIt->second.size(); ++ i)
                {
                    valid &= getDoubleVal(pidIt->second[i], gains[i]);
                }

                if (valid)
                {
                    ROS_DEBUG("setVelPid: %s, %f, %f, %f, %f, %f, %f, %f", pidIt->first.c_str(), gains[0], gains[1], gains[2], gains[3], gains[4], gains[5], gains[6]);
                    robotControllerPtr->setVelPid(pidIt->first, gains[0], gains[1], gains[2], gains[3], gains[4], gains[5], gains[6]);
                }
                else
                {
                    ROS_WARN("velocity PID for %s not set because the values are not valid (double or int)", pidIt->first.c_str());
                }
            }
            else
            {
                ROS_WARN("velocity PID values for %s not set because it is not an array of size 3 or greater", pidIt->first.c_str());
            }
        }
    }

    // if not in advanced mode, tell the controller to hold the joint positions
    if (!advancedMode)
    {
        physics::Joint_V _joints = modelPtr->GetJoints();
        for (unsigned int i = 0; i < modelPtr->GetJointCount(); ++i)
        {
            physics::JointPtr jPtr = _joints[i];
            robotControllerPtr->setJointPosTarget(jPtr->GetName(), jPtr->Position(0));
        }
    }

    // subscribe to ROS joint commands
    jointCommandsSub  = rosNodePtr->subscribe(jointCommandsTopic,  5, &GazeboInterface::commandJoints,  this);

    // advertise joint state publishing
    jointStatePub = rosNodePtr->advertise<sensor_msgs::JointState>(jointStatesTopic, true);

    // advertise joint capabilities publishing
    jointCapabilitiesPub = rosNodePtr->advertise<nasa_r2_common_msgs::JointCapability>(jointCapabilitiesTopic, true);

    // advertise joint command ref publishing
    jointCommandRefsPub = rosNodePtr->advertise<nasa_r2_common_msgs::JointCommand>(jointCommandRefsTopic, true);

    if (advancedMode)
    {
        // subscribe to ROS joint control messages
        jointControlSub  = rosNodePtr->subscribe(jointControlTopic,  5, &GazeboInterface::controlJoints,  this);

        // advertise joint status publishing
        jointStatusPub = rosNodePtr->advertise<nasa_r2_common_msgs::JointStatusArray>(jointStatusTopic, true);
    }

    ROS_INFO("Gazebo Interface plugin initialized");
}

void GazeboInterface::update()
{
    common::Time currTime = modelPtr->GetWorld()->SimTime();

    // update joint controller
    robotControllerPtr->update();

    //publish joint states
    if ((currTime - prevStatesUpdateTime).Double() >= jointStatesStepTime)
    {
        prevStatesUpdateTime = currTime;
        sensor_msgs::JointStatePtr msgPtr(new sensor_msgs::JointState);
        nasa_r2_common_msgs::JointCommandPtr refPtr(new nasa_r2_common_msgs::JointCommand);
        nasa_r2_common_msgs::JointCapabilityPtr capPtr(new nasa_r2_common_msgs::JointCapability);
        msgPtr->header.stamp = ros::Time::now();
        refPtr->header.stamp = ros::Time::now();

        // get positions
        robotControllerPtr->getJointStates(jointStates);

        // build position message
        for (std::map<std::string, RobotController::JointState>::const_iterator stateIt = jointStates.begin(); stateIt != jointStates.end(); ++stateIt)
        {
            std::string::size_type index = stateIt->first.find("fixed");
            if (index != std::string::npos)
            {
                continue;
            }
            index = stateIt->first.find("/joint");
            if (index != std::string::npos)
            {
                std::string name = stateIt->first;
                msgPtr->name.push_back(name);
                msgPtr->position.push_back(stateIt->second.position);
                msgPtr->velocity.push_back(stateIt->second.velocity);
                msgPtr->effort.push_back(stateIt->second.effort);
            }
            else
            {
                msgPtr->name.push_back(stateIt->first);
                msgPtr->position.push_back(stateIt->second.position);
                msgPtr->velocity.push_back(stateIt->second.velocity);
                msgPtr->effort.push_back(stateIt->second.effort);
            }
        }

        // get targets
        robotControllerPtr->getJointTargets(jointCommandRefs);

        // build targets message
        for (std::map<std::string, RobotController::JointState>::const_iterator refsIt = jointCommandRefs.begin(); refsIt != jointCommandRefs.end(); ++refsIt)
        {
            std::string::size_type index = refsIt->first.find("fixed");
            if (index == std::string::npos)
            {
                refPtr->name.push_back(refsIt->first);
                refPtr->desiredPosition.push_back(refsIt->second.position);
                refPtr->desiredPositionVelocityLimit.push_back(refsIt->second.velocity);
                refPtr->feedForwardTorque.push_back(refsIt->second.effort);
                refPtr->proportionalGain.push_back(0);
                refPtr->derivativeGain.push_back(0);
                refPtr->integralGain.push_back(0);
                refPtr->positionLoopTorqueLimit.push_back(0);
                refPtr->positionLoopWindupLimit.push_back(0);
                refPtr->torqueLoopVelocityLimit.push_back(0);
            }
        }

        // get capabilities
        robotControllerPtr->getJointLimits(jointLimits);

        // build capability message
        for (std::map<std::string, std::pair<double, double> >::const_iterator limsIt = jointLimits.begin(); limsIt != jointLimits.end(); ++limsIt)
        {
            std::string::size_type index = limsIt->first.find("fixed");
            if (index == std::string::npos)
            {
                capPtr->name.push_back(limsIt->first);
                capPtr->positionLimitMax.push_back(limsIt->second.second);
                capPtr->positionLimitMin.push_back(limsIt->second.first);
                capPtr->torqueLimit.push_back(1000.);
            }
        }

	jointCommandRefsPub.publish(refPtr);
        jointStatePub.publish(msgPtr);
        //jointCommandRefsPub.publish(refPtr);
        jointCapabilitiesPub.publish(capPtr);
    }

    //publish joint status
    if (advancedMode && (currTime - prevStatusUpdateTime).Double() >= jointStatusStepTime)
    {
        prevStatusUpdateTime = currTime;
        robotControllerPtr->publishJointStatuses(jointStatusPub);
    }
}

void GazeboInterface::commandJoints(const sensor_msgs::JointState::ConstPtr& msg)
{
    bool setPos = msg->position.size() >= msg->name.size();
    bool setVel  = msg->velocity.size() >= msg->name.size();
    bool setEffort  = msg->effort.size() >= msg->name.size();

    ROS_DEBUG("GazeboInterface received joint command");
    for (unsigned int i = 0; i < msg->name.size(); ++i)
    {
        if (setPos)
        {
            robotControllerPtr->setJointPosTarget(msg->name[i], msg->position[i]);
        }
        else if (setEffort)
        {
            robotControllerPtr->setJointEffortTarget(msg->name[i], msg->effort[i]);
        }
        else if (setVel)
        {
            robotControllerPtr->setJointVelTarget(msg->name[i], msg->velocity[i]);
        }
    }
}

// handle control message
void GazeboInterface::controlJoints(const nasa_r2_common_msgs::JointControl::ConstPtr& msg)
{
    ROS_DEBUG("GazeboInterface received joint control");
    robotControllerPtr->setJointControl(msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboInterface)
