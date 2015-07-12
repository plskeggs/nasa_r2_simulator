#include <ros/ros.h>
#include <nasa_robodyn_ros/R2RosTrajectoryManager.hpp>
#include <nasa_robodyn_ros/R2RosArbiter.hpp>
#include <nasa_robodyn_ros/R2RosTreeId.hpp>
#include <nasa_robodyn_ros/R2RosGripper.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "R2TrajectoryManager");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    double rate;
    std::vector<double> priorityLevels;
    std::vector<double> linearTols;
    std::vector<double> angularTols;
    double momLimit;
    double taskGain;
    int ikMaxItr;
    double ikMaxTwist;

    std::string controlUrdf;
    std::string dynamicUrdf;

    pnh.param("rate", rate, 30.0);
    //! @todo make these a parameter
    priorityLevels.push_back(1);
    priorityLevels.push_back(2);
    priorityLevels.push_back(3);
    priorityLevels.push_back(4);
    linearTols.push_back(0.0001);
    linearTols.push_back(0.001);
    linearTols.push_back(0.01);
    linearTols.push_back(0.1);
    angularTols.push_back(0.000174533);
    angularTols.push_back(0.001745329);
    angularTols.push_back(0.017453293);
    angularTols.push_back(0.174532925);
    momLimit = 1e-12;
    taskGain = 000032;
    ikMaxItr = 10;
    ikMaxTwist = 0.1;
    pnh.getParam("control_urdf", controlUrdf);
    pnh.getParam("dynamic_urdf", dynamicUrdf);
    R2RosTrajectoryManager trajManager(ros::this_node::getName(), controlUrdf, 1.0 / rate, pnh);
    trajManager.setCartesianParameters(priorityLevels, linearTols, angularTols, momLimit, taskGain, ikMaxItr, ikMaxTwist);

    double x, y, z;
    std::string idMode;
    std::string baseFrame;
    nh.param("/gazebo/gravity_x", x, 0.0);
    nh.param("/gazebo/gravity_y", y, 0.0);
    nh.param("/gazebo/gravity_z", z, -9.8);
    if(x == 0 && y == 0 && z == 0)
    {
        pnh.param("idMode", idMode, std::string("inertia"));
    }
    else
    {
        pnh.param("idMode", idMode, std::string("full"));
    }
    pnh.param("baseFrame", baseFrame, std::string("r2/robot_world"));
    R2RosTreeId treeId(controlUrdf, pnh);
    treeId.setGravity(x, y, z, baseFrame, idMode);

    R2RosArbiter arbiter(pnh);

    //! set up the gripper
    R2RosGripper gripper(pnh);

    XmlRpc::XmlRpcValue gripper_list;
    std::vector<std::string> gripperNames;
    pnh.param<XmlRpc::XmlRpcValue>("gripper/names", gripper_list, XmlRpc::XmlRpcValue());
    if(gripper_list.valid())
    {
        ROS_ASSERT(gripper_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int32_t i = 0; i < gripper_list.size(); ++i)
        {
            ROS_ASSERT(gripper_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
            gripperNames.push_back(static_cast<std::string>(gripper_list[i]));
        }
    }
    else
    {
        gripperNames.push_back("r2/left_leg/gripper/joint0");
        gripperNames.push_back("r2/right_leg/gripper/joint0");
    }

    double gripperVel;
    pnh.param<double>("gripper/velocity", gripperVel, 1.);


    double gripperAcc;
    pnh.param<double>("gripper/acceleration", gripperAcc, 1.);

    gripper.configure(gripperNames, gripperVel, gripperAcc);

    ros::Rate r(rate);

    while (nh.ok())
    {
        //! update hook
        trajManager.update();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
