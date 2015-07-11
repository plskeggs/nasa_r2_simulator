#include "r2_controllers_ros/R2RosArbiter.hpp"

R2RosArbiter::R2RosArbiter(ros::NodeHandle nh)
{
    nh = nh;
    modeIn                     = nh.subscribe("/mode", 1, &R2RosArbiter::modeCallback, this);
    labeledJointSettingsIn     = nh.subscribe("/joint_ref_settings", 1, &R2RosArbiter::labeledJointSettingsCallback, this);
    labeledPoseSettingsIn      = nh.subscribe("/pose_ref_settings", 1, &R2RosArbiter::labeledPoseSettingsCallback, this);
    labeledJointStateIn        = nh.subscribe("/desired_torque_limits", 1, &R2RosArbiter::labeledJointStateCallback, this);
    labeledGainsIn             = nh.subscribe("/joint_desired_dynamics", 1, &R2RosArbiter::labeledGainsCallback, this);
    labeledJointTrajIn         = nh.subscribe("/joint_refs", 32, &R2RosArbiter::labeledJointTrajectoryCallback, this);
    labeledPoseTrajIn          = nh.subscribe("/pose_refs", 32, &R2RosArbiter::labeledPoseTrajectoryCallback, this);

    jointSettingsOut = nh.advertise<nasa_r2_common_msgs::ControllerJointSettings>("joint_ref_settings", 1);
    poseSettingsOut  = nh.advertise<nasa_r2_common_msgs::ControllerPoseSettings>("pose_ref_settings", 1);
    jointStateOut    = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    gainsOut         = nh.advertise<nasa_r2_common_msgs::Gains>("gains", 1);
    jointTrajOut     = nh.advertise<nasa_r2_common_msgs::JointTrajectoryReplan>("joint_refs", 1);
    poseTrajOut      = nh.advertise<nasa_r2_common_msgs::PoseTrajectoryReplan>("pose_refs", 10);

    goalStatusOut    = nh.advertise<actionlib_msgs::GoalStatusArray>("/arbiter_goal_status", 1);
}

void R2RosArbiter::writeSuccessStatus(std::string id)
{
    actionlib_msgs::GoalStatusArray goalStatusMsg;
    actionlib_msgs::GoalStatus gStatus;
    gStatus.status = actionlib_msgs::GoalStatus::SUCCEEDED;
    gStatus.text = "Successful command pass-thru";
    gStatus.goal_id.id = id;

    goalStatusMsg.status_list.push_back(gStatus);
    goalStatusOut.publish(goalStatusMsg);
}

void R2RosArbiter::modeCallback(const nasa_r2_common_msgs::Modes &msg)
{
    writeSuccessStatus("Mode");
}

void R2RosArbiter::labeledJointSettingsCallback(const nasa_r2_common_msgs::LabeledControllerJointSettings &msg)
{
    nasa_r2_common_msgs::ControllerJointSettings passthrough;
    passthrough.joint_names = msg.joint_names;
    passthrough.jointVelocityLimits = msg.jointVelocityLimits;
    passthrough.jointAccelerationLimits = msg.jointAccelerationLimits;
    jointSettingsOut.publish(passthrough);

    writeSuccessStatus("JointSettings");
}

void R2RosArbiter::labeledPoseSettingsCallback(const nasa_r2_common_msgs::LabeledControllerPoseSettings &msg)
{
    nasa_r2_common_msgs::ControllerPoseSettings passthrough;
    passthrough.maxLinearAcceleration = msg.maxLinearAcceleration;
    passthrough.maxLinearVelocity = msg.maxLinearVelocity;
    passthrough.maxRotationalAcceleration = msg.maxRotationalAcceleration;
    passthrough.maxRotationalVelocity = msg.maxRotationalVelocity;
    poseSettingsOut.publish(passthrough);
    writeSuccessStatus("PoseSettings");
}

void R2RosArbiter::labeledJointStateCallback(const nasa_r2_common_msgs::LabeledJointState &msg)
{
    sensor_msgs::JointState passthrough;
    passthrough.effort = msg.effort;
    passthrough.header = msg.header;
    passthrough.name = msg.name;
    passthrough.position = msg.position;
    passthrough.velocity = msg.velocity;
    jointStateOut.publish(passthrough);
    writeSuccessStatus("JointTorqueLimits");
}

void R2RosArbiter::labeledGainsCallback(const nasa_r2_common_msgs::LabeledGains &msg)
{
    nasa_r2_common_msgs::Gains passthrough;
    passthrough.D = msg.D;
    passthrough.I = msg.I;
    passthrough.joint_names = msg.joint_names;
    passthrough.K = msg.K;
    passthrough.naturalFreq = msg.naturalFreq;
    passthrough.windupLimit = msg.windupLimit;
    gainsOut.publish(passthrough);
    writeSuccessStatus("Gains");
}

void R2RosArbiter::labeledJointTrajectoryCallback(const nasa_r2_common_msgs::LabeledJointTrajectory &msg)
{
    nasa_r2_common_msgs::JointTrajectoryReplan passthrough;
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.header = msg.header;
    trajectory.joint_names = msg.joint_names;
    trajectory.points = msg.points;
    passthrough.header = msg.header;
    passthrough.trajectory = trajectory;
    jointTrajOut.publish(passthrough);
    writeSuccessStatus("JointRef");
}

void R2RosArbiter::labeledPoseTrajectoryCallback(const nasa_r2_common_msgs::LabeledPoseTrajectory &msg)
{
    ROS_INFO("got labeled pose traj");
    nasa_r2_common_msgs::PoseTrajectoryReplan passthrough;
    nasa_r2_common_msgs::PoseTrajectory trajectory;
    trajectory.header = msg.header;
    trajectory.nodes = msg.nodes;
    trajectory.node_priorities = msg.node_priorities;
    trajectory.points = msg.points;
    trajectory.refFrames = msg.refFrames;
    passthrough.header = msg.header;
    passthrough.trajectory = trajectory;
    poseTrajOut.publish(passthrough);
    ROS_INFO("wrote posetraj");
    writeSuccessStatus("PoseTraj");
}
