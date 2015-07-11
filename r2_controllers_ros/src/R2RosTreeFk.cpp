#include <r2_controllers_ros/R2RosTreeFk.hpp>

R2RosTreeFk::R2RosTreeFk(std::string urdf)
{
    treeFk.loadFromFile(urdf);
}

void R2RosTreeFk::setup(std::string inputJointStatesName, std::string outputPoseStatesName)
{
    //! Publishers and subscribers
    poseStatesOut = nh.advertise<nasa_r2_common_msgs::PoseState>(outputPoseStatesName, 1);
    jointStatesIn = nh.subscribe(inputJointStatesName, 1, &R2RosTreeFk::jointStatesCallback, this);
}

void R2RosTreeFk::jointStatesCallback(const sensor_msgs::JointState &msg)
{
    treeFk.getPoseState(msg, poseStatesOutMsg);
    poseStatesOut.publish(poseStatesOutMsg);
}
