#include <ros/ros.h>
#include "nasa_r2_common_msgs/LabeledControllerJointSettings.h"
#include "nasa_r2_common_msgs/LabeledControllerPoseSettings.h"
#include "nasa_r2_common_msgs/LabeledJointState.h"
#include "nasa_r2_common_msgs/LabeledGains.h"
#include "nasa_r2_common_msgs/LabeledJointTrajectory.h"
#include "nasa_r2_common_msgs/LabeledPoseTrajectory.h"
#include "nasa_r2_common_msgs/LabeledGripperPositionCommand.h"
#include "nasa_r2_common_msgs/Modes.h"

#include "nasa_r2_common_msgs/ControllerJointSettings.h"
#include "nasa_r2_common_msgs/ControllerPoseSettings.h"
#include "sensor_msgs/JointState.h"
#include "nasa_r2_common_msgs/Gains.h"
#include "nasa_r2_common_msgs/JointTrajectoryReplan.h"
#include "nasa_r2_common_msgs/PoseTrajectoryReplan.h"

#include "actionlib_msgs/GoalStatusArray.h"

/** 
  * R2RosArbiter
  *******************************************************************************/
class R2RosArbiter
{
public:
    R2RosArbiter(ros::NodeHandle nh);
    virtual ~R2RosArbiter() {}

    void modeCallback(const nasa_r2_common_msgs::Modes& msg);
    
    void labeledJointSettingsCallback(const nasa_r2_common_msgs::LabeledControllerJointSettings& msg);
    void labeledPoseSettingsCallback(const nasa_r2_common_msgs::LabeledControllerPoseSettings& msg);
    void labeledJointStateCallback(const nasa_r2_common_msgs::LabeledJointState& msg);
    void labeledGainsCallback(const nasa_r2_common_msgs::LabeledGains& msg);
    void labeledJointTrajectoryCallback(const nasa_r2_common_msgs::LabeledJointTrajectory& msg);
    void labeledPoseTrajectoryCallback(const nasa_r2_common_msgs::LabeledPoseTrajectory& msg);

private:
    void writeSuccessStatus(std::string id);

    ros::NodeHandle nh;

    ros::Subscriber modeIn;
    ros::Subscriber labeledJointSettingsIn;
    ros::Subscriber labeledPoseSettingsIn;
    ros::Subscriber labeledJointStateIn;
    ros::Subscriber labeledGainsIn;
    ros::Subscriber labeledJointTrajIn;
    ros::Subscriber labeledPoseTrajIn;
    ros::Subscriber labeledGripperCommandIn;
    
    ros::Publisher jointSettingsOut;
    ros::Publisher poseSettingsOut;
    ros::Publisher jointStateOut;
    ros::Publisher gainsOut;
    ros::Publisher jointTrajOut;
    ros::Publisher poseTrajOut;

    ros::Publisher goalStatusOut;
};
