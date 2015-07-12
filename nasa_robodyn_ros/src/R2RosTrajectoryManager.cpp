#include "nasa_robodyn_ros/R2RosTrajectoryManager.hpp"

R2RosTrajectoryManager::R2RosTrajectoryManager(const string name, const string& urdf, const double& timestep, ros::NodeHandle& nh)
    : TrajectoryManager(name)
{
    urdfFile = urdf;

    initialize(urdf, timestep);

    treeFk.loadFromFile(urdf);

    this->nh = nh;

    //! publishers and subscribers
    jointCommandOut = nh.advertise<sensor_msgs::JointState>("/joint_desired", 1);
    poseStatesOut   = nh.advertise<nasa_r2_common_msgs::PoseState>("/pose_states", 1);
    poseCommandOut  = nh.advertise<nasa_r2_common_msgs::PoseState>("/pose_command_refs", 1);
    goalStatusOut   = nh.advertise<actionlib_msgs::GoalStatusArray>("/goal_status", 10);
    jointSettingsOut = nh.advertise<nasa_r2_common_msgs::ControllerJointSettings>("/joint_settings", 10);

    jointSettingsIn     = nh.subscribe("joint_ref_settings", 1, &R2RosTrajectoryManager::jointSettingsCallback, this);
    jointCommandRefsIn  = nh.subscribe("/joint_command_refs", 1, &R2RosTrajectoryManager::jointCommandCallback, this);
    jointStatesIn       = nh.subscribe("/joint_states", 1, &R2RosTrajectoryManager::jointStatesCallback, this);
    jointCapabilitiesIn = nh.subscribe("/joint_capabilities", 1, &R2RosTrajectoryManager::jointCapabilitiesCallback, this);
    jointRefsIn         = nh.subscribe("joint_refs", 32, &R2RosTrajectoryManager::jointRefsCallback, this);
    jointTrajIn         = nh.subscribe("joint_traj", 32, &R2RosTrajectoryManager::jointTrajCallback, this);

    basesIn             = nh.subscribe("/base_frames", 1, &R2RosTrajectoryManager::basesCallback, this);
    inertiaIn           = nh.subscribe("/treeid_inertia", 1, &R2RosTrajectoryManager::inertiaCallback, this);
    poseSettingsIn      = nh.subscribe("pose_ref_settings", 1, &R2RosTrajectoryManager::poseSettingsCallback, this);
    poseStatesIn        = nh.subscribe("/pose_states", 1, &R2RosTrajectoryManager::poseStatesCallback, this);
    poseCommandRefIn    = nh.subscribe("/pose_command_refs", 1, &R2RosTrajectoryManager::poseCommandCallback, this);
    poseRefsIn          = nh.subscribe("pose_refs", 1, &R2RosTrajectoryManager::poseRefsCallback, this);

    poseRefsOut         = nh.advertise<nasa_r2_common_msgs::PoseTrajectoryReplan>("pose_refs", 10);
}

void R2RosTrajectoryManager::setCartesianParameters(std::vector<double> priorityNum, std::vector<double> linearTol, std::vector<double> angularTol, double momLimit, double taskGain, int ikMaxItr, double ikMaxTwist)
{
    setPriorityTol(priorityNum, linearTol, angularTol);
    setIkParameters(momLimit, taskGain, ikMaxItr, ikMaxTwist);
    nasa_r2_common_msgs::StringArray msg;
    msg.data.push_back("r2/robot_world");
    setBases(msg);
}

void R2RosTrajectoryManager::jointSettingsCallback(const nasa_r2_common_msgs::ControllerJointSettings& msg)
{
    ROS_INFO("in JointSettingsCallback");
    setJointSettings(msg);
    getJointSettings(jointSettingsMsg);
    jointSettingsOut.publish(jointSettingsMsg);
}

void R2RosTrajectoryManager::jointCommandCallback(const nasa_r2_common_msgs::JointCommand& msg)
{
    ROS_DEBUG("in JointCommandCallback");
    prevJointCommandsMsg = jointCommandsMsg;
    jointCommandsMsg     = msg;
    treeFk.getPoseState(fromJointCommandToJointState(msg), poseCommandOutMsg);
    poseCommandOut.publish(poseCommandOutMsg);
}

void R2RosTrajectoryManager::jointStatesCallback(const sensor_msgs::JointState& msg)
{
    ROS_DEBUG("in JointStatesCallback");
    prevJointStatesMsg = jointStatesMsg;
    jointStatesMsg     = msg;
    treeFk.getPoseState(msg, poseStatesOutMsg);
    poseStatesOut.publish(poseStatesOutMsg);
}

void R2RosTrajectoryManager::jointCapabilitiesCallback(const nasa_r2_common_msgs::JointCapability& msg)
{
    ROS_DEBUG("in JointCapabilitiesCallback");
    setJointCapabilities(msg);
}

void R2RosTrajectoryManager::jointRefsCallback(const nasa_r2_common_msgs::JointTrajectoryReplan& jointRefsMsg)
{
    ROS_INFO("Joint Ref Trajectory received");

    if (jointRefsMsg.trajectory.joint_names.empty())
    {
        ROS_INFO("Stopping all joints -- joint names empty");
        kasquishAll(fromJointCommandToJointState(jointCommandsMsg), fromJointCommandToJointState(jointCommandsMsg), "new joint refs with no joint names");
    }
    else
    {
        ROS_INFO("added joint waypoints");
        addJointWaypoints(jointRefsMsg.trajectory, fromJointCommandToJointState(jointCommandsMsg), fromJointCommandToJointState(jointCommandsMsg), fromJointCommandToJointState(prevJointCommandsMsg));
    }
}

void R2RosTrajectoryManager::jointTrajCallback(const trajectory_msgs::JointTrajectory& trajInMsg)
{
    ROS_INFO("in JointTrajCallback");

    if (trajInMsg.joint_names.empty())
    {
        ROS_INFO("Stopping all joints -- joint names empty");
        kasquishAll(fromJointCommandToJointState(jointCommandsMsg), fromJointCommandToJointState(jointCommandsMsg), "new joint trajectory with no joint names");
    }
    else
    {
        addJointBreadcrumbs(trajInMsg);
    }
}

void R2RosTrajectoryManager::basesCallback(const nasa_r2_common_msgs::StringArray& msg)
{
    ROS_INFO("set new bases");
    setBases(msg);
}

void R2RosTrajectoryManager::inertiaCallback(const sensor_msgs::JointState& msg)
{
    ROS_DEBUG("in inertiaCallback");
    updateInertia(msg);
}

void R2RosTrajectoryManager::poseSettingsCallback(const nasa_r2_common_msgs::ControllerPoseSettings& msg)
{
    ROS_INFO("in poseSettingsCallback");
    setPoseSettings(msg);
}

void R2RosTrajectoryManager::poseStatesCallback(const nasa_r2_common_msgs::PoseState& msg)
{
    ROS_DEBUG("in poseStatesCallback");
    updateActualPoseState(msg);
}

void R2RosTrajectoryManager::poseCommandCallback(const nasa_r2_common_msgs::PoseState& msg)
{
    ROS_DEBUG("in poseCommandCallback");
    poseCommandsMsg = msg;
}

void R2RosTrajectoryManager::poseRefsCallback(const nasa_r2_common_msgs::PoseTrajectoryReplan& msg)
{
    ROS_INFO("adding new pose in poseRefsCallback");
    poseRefsMsg = msg;

    double waitTime = addCartesianWaypoints(poseRefsMsg.trajectory, fromJointCommandToJointState(jointCommandsMsg), fromJointCommandToJointState(jointCommandsMsg), fromJointCommandToJointState(prevJointCommandsMsg), poseCommandsMsg, poseCommandsMsg);

    if (waitTime > 0)
    {
        ROS_INFO("waiting for kasquish to finish");
        ros::Duration(waitTime).sleep();
        poseRefsOut.publish(msg);
        return;
    }

    ROS_INFO("Adding cartesian trajectory");
}

void R2RosTrajectoryManager::writeStatus(const actionlib_msgs::GoalStatusArray& goalStatusMsg_out)
{
    ROS_INFO("wrote status");
    goalStatusOut.publish(goalStatusMsg_out);
}

void R2RosTrajectoryManager::writeJointState(const sensor_msgs::JointState& jointState_out)
{
    ROS_DEBUG("wrote joint command");
    jointCommandOutMsg = jointState_out;
    jointCommandOut.publish(jointCommandOutMsg);

}

void R2RosTrajectoryManager::writeJointControl(const nasa_r2_common_msgs::JointControlDataArray& jointControl_out)
{
}

void R2RosTrajectoryManager::update()
{
    getJointSettings(jointSettingsMsg);
    jointSettingsOut.publish(jointSettingsMsg);

    try
    {
        updateTrajectories(jointCommandsMsg, fromJointCommandToJointControl(jointCommandsMsg), fromJointCommandToJointState(jointCommandsMsg), fromJointCommandToJointState(jointCommandsMsg));
    }
    catch (std::exception& e)
    {
        ROS_ERROR("TrajectoryManagerComponent: something crazy happened in updateTrajectories, moving to error state");
    }
}

sensor_msgs::JointState R2RosTrajectoryManager::fromJointCommandToJointState(const nasa_r2_common_msgs::JointCommand& jointCommandMsg)
{
    sensor_msgs::JointState jointStateMsg;

    jointStateMsg.header   = jointCommandMsg.header;
    jointStateMsg.name     = jointCommandMsg.name;
    jointStateMsg.position = jointCommandMsg.desiredPosition;
    jointStateMsg.velocity = jointCommandMsg.desiredPositionVelocityLimit;
    jointStateMsg.effort   = jointCommandMsg.feedForwardTorque;

    return jointStateMsg;
}

nasa_r2_common_msgs::JointControlDataArray R2RosTrajectoryManager::fromJointCommandToJointControl(const nasa_r2_common_msgs::JointCommand& jointCommandMsg)
{
    nasa_r2_common_msgs::JointControlDataArray jointControlMsg;

    if (jointControlMsg.joint != jointCommandMsg.name)
    {
        jointControlMsg.joint.clear();
        jointControlMsg.data.clear();

        for (unsigned int i = 0; i < jointCommandMsg.name.size(); ++i)
        {
            jointControlMsg.joint.push_back(jointCommandMsg.name[i]);
            nasa_r2_common_msgs::JointControlData jc;
            jc.calibrationMode.state = nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE;
            jc.clearFaultMode.state  = nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE;
            jc.coeffState.state      = nasa_r2_common_msgs::JointControlCoeffState::LOADED;
            jc.commandMode.state     = nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH;
            jc.controlMode.state     = nasa_r2_common_msgs::JointControlMode::DRIVE;
            jointControlMsg.data.push_back(jc);
        }
    }

    return jointControlMsg;
}
