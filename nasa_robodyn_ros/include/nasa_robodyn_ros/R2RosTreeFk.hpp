#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "nasa_r2_common_msgs/PoseState.h"
#include "nasa_robodyn_controllers_core/RosMsgTreeFk.h"

class R2RosTreeFk
{
public:
    R2RosTreeFk(std::string urdf);
    virtual ~R2RosTreeFk(){}
    void setup(std::string inputJointStatesName, std::string outputPoseStatesName);
    void jointStatesCallback(const sensor_msgs::JointState& msg);

private:
    ros::Publisher  poseStatesOut;
    ros::Subscriber jointStatesIn;

    nasa_r2_common_msgs::PoseState poseStatesOutMsg;

    RosMsgTreeFk treeFk;
};
