#include <ros/ros.h>
#include <queue>

#include "nasa_robodyn_controllers_core/TrajectoryManager.h"
#include "nasa_robodyn_controllers_core/RosMsgTreeFk.h"

#include "sensor_msgs/JointState.h"
#include "nasa_r2_common_msgs/JointCapability.h"
#include "nasa_r2_common_msgs/JointCommand.h"
#include "nasa_r2_common_msgs/JointControlDataArray.h"
#include "nasa_r2_common_msgs/JointTrajectoryReplan.h"
#include "nasa_r2_common_msgs/PoseTrajectoryReplan.h"
#include "nasa_r2_common_msgs/ForceControlAxisArray.h"
#include "nasa_r2_common_msgs/StringArray.h"

class R2RosTrajectoryManager : public TrajectoryManager
{
    public:
        R2RosTrajectoryManager(const std::string name, const std::string& urdf, const double& timestep, ros::NodeHandle &nh);
        virtual ~R2RosTrajectoryManager() {}

        void setCartesianParameters(std::vector<double> priorityNum, std::vector<double> linearTol, std::vector<double> angularTol, double momLimit, double taskGain, int ikMaxItr, double ikMaxTwist);
        
        void jointSettingsCallback(const nasa_r2_common_msgs::ControllerJointSettings& msg);
        void jointCommandCallback(const nasa_r2_common_msgs::JointCommand& msg);
        void jointStatesCallback(const sensor_msgs::JointState& msg);
        void jointCapabilitiesCallback(const nasa_r2_common_msgs::JointCapability& msg);
        void jointRefsCallback(const nasa_r2_common_msgs::JointTrajectoryReplan& msg);
        void jointTrajCallback(const trajectory_msgs::JointTrajectory& msg);

        void basesCallback(const nasa_r2_common_msgs::StringArray& msg);
        void inertiaCallback(const sensor_msgs::JointState& msg);
        void poseSettingsCallback(const nasa_r2_common_msgs::ControllerPoseSettings& msg);
        void poseStatesCallback(const nasa_r2_common_msgs::PoseState& msg);
        void poseCommandCallback(const nasa_r2_common_msgs::PoseState& msg);
        void poseRefsCallback(const nasa_r2_common_msgs::PoseTrajectoryReplan& msg);
        
        void update();
        sensor_msgs::JointState fromJointCommandToJointState(const nasa_r2_common_msgs::JointCommand& jointCommandMsg);
        nasa_r2_common_msgs::JointControlDataArray fromJointCommandToJointControl(const nasa_r2_common_msgs::JointCommand& jointCommandMsg);
        
    protected:
        virtual void writeStatus(const actionlib_msgs::GoalStatusArray& goalStatusMsg_out);

        virtual void writeJointState(const sensor_msgs::JointState& jointState_out);
        virtual void writeJointControl(const nasa_r2_common_msgs::JointControlDataArray& jointControl_out);
        
    private:
        //! publishers and subscribers
        ros::Publisher jointCommandOut;
        ros::Publisher poseCommandOut;
        ros::Publisher poseStatesOut;
        ros::Publisher goalStatusOut;
        
        ros::Subscriber jointSettingsIn;
        ros::Subscriber jointCommandRefsIn;
        ros::Subscriber jointStatesIn;
        ros::Subscriber jointCapabilitiesIn;
        ros::Subscriber jointRefsIn;
        ros::Subscriber jointTrajIn;

        ros::Subscriber basesIn;
        ros::Subscriber inertiaIn;
        ros::Subscriber poseSettingsIn;
        ros::Subscriber poseStatesIn;
        ros::Subscriber poseCommandRefIn;
        ros::Subscriber poseRefsIn;
    
        //! messages
        nasa_r2_common_msgs::ControllerJointSettings jointSettingsMsg;
        nasa_r2_common_msgs::JointTrajectoryReplan   jointRefsMsg;
        sensor_msgs::JointState                      prevJointStatesMsg;
        sensor_msgs::JointState                      jointStatesMsg;
        nasa_r2_common_msgs::JointCommand            jointCommandsMsg;
        nasa_r2_common_msgs::JointCommand            prevJointCommandsMsg;
        nasa_r2_common_msgs::JointControlDataArray   jointStatusMsg;
        trajectory_msgs::JointTrajectory             trajInMsg;
        sensor_msgs::JointState                      jointCommandOutMsg;

        nasa_r2_common_msgs::PoseState                        poseCommandsMsg;
        nasa_r2_common_msgs::PoseTrajectoryReplan             poseRefsMsg;
        std::queue<nasa_r2_common_msgs::PoseTrajectoryReplan> poseRefsQueue;
        nasa_r2_common_msgs::PoseState                        poseStatesOutMsg;
        nasa_r2_common_msgs::PoseState                        poseCommandOutMsg;

        std::string urdfFile;
        double timeStepProp;
        double waitForKasquish;
        
        bool newJointCommands;
        bool newPoseCommands;
        
        nasa_r2_common_msgs::ControllerJointSettings replanJointSettings;
        double replanHoldVelocityLimit;
        double replanHoldAccelerationLimit;
        double replanStopVelocityLimit;
        double replanStopAccelerationLimit;
        double replanSettingsVelocityCutoff;

        RosMsgTreeFk treeFk;
        
};
