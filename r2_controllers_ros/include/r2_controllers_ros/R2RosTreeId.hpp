#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include "nasa_r2_common_msgs/WrenchState.h"
#include "nasa_r2_common_msgs/StringArray.h"

#include "nasa_robodyn_controllers_core/RosMsgTreeId.h"
#include "nasa_robodyn_controllers_core/JointDynamicsData.h"
#include "nasa_robodyn_controllers_core/KdlTreeId.h"
#include "nasa_robodyn_utilities/RosMsgConverter.h"

class R2RosTreeId
{
public:
    R2RosTreeId(std::string urdf, ros::NodeHandle &nh);
    virtual ~R2RosTreeId(){}

    void baseFrameCallback(const nasa_r2_common_msgs::StringArray &msg);
    void forceCallback(const nasa_r2_common_msgs::WrenchState &msg);
    void jointStateCallback(const sensor_msgs::JointState &msg);
    void trajCallback(const sensor_msgs::JointState &msg);
    void imuCallback(const sensor_msgs::Imu &msg);

    bool setGravity(double x, double y, double z, std::string frame, std::string mode);

private:
    // Inv Dyn objects
    RosMsgTreeId rmt;
    KdlTreeId id;
    JointDynamicsData jd;

    // messages
    nasa_r2_common_msgs::StringArray baseMsg, completionMsg;
    nasa_r2_common_msgs::WrenchState forceMsg, emptyForceMsg, segForceMsg;
    sensor_msgs::JointState          actualStateMsg;
    sensor_msgs::JointState          trajStateMsg, emptyTrajMsg;
    sensor_msgs::JointState          jointTorqueMsg;
    sensor_msgs::JointState          jointInertiaMsg;
    sensor_msgs::Imu                 imuInMsg;

    //Publishers/Subscribers

    ros::Subscriber baseFrameIn;
    ros::Subscriber forceIn;
    ros::Subscriber jointStateIn;
    ros::Subscriber trajIn;
    ros::Subscriber imuIn;

    ros::Publisher torqueOut;
    ros::Publisher inertiaOut;
    ros::Publisher segForceOut;
    ros::Publisher completionOut;

    // properties
    std::vector<double> gravity;
    KDL::Vector gravity_kdl;
    std::string gravityFrame;
    KDL::Frame gravityXform;
    bool argos, useImu;
    std::string gravityBase, idMode;
    double yankLimBFF, loopRate;

    // variables for the id process
    std::string baseFrame, bfCmd;
    double bfFactor;
    KDL::Twist v_in, a_in, zeroTwist;
    KDL::Wrench f_out;
    KDL::RigidBodyInertia I_out;
    bool newBase;

};
