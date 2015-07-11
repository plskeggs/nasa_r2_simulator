#include <ros/ros.h>
#include <r2_controllers_ros/R2RosTrajectoryManager.hpp>
#include <r2_controllers_ros/R2RosArbiter.hpp>
#include <r2_controllers_ros/R2RosTreeId.hpp>


int main(int argc, char **argv)
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

    std::string urdf;

    pnh.param("rate", rate, 30.0);
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
//    pnh.param("priorityLevels", priorityLevels, std::vector<double>(1,1.0));
//    pnh.param("linearTols", linearTols, std::vector<double>(1, 0.00001));
//    pnh.param("angularTols", angularTols, std::vector<double>(1, 0.00001));
//    pnh.param("singularityLimit", momLimit, 1e-12);
//    pnh.param("singularityGain", taskGain, 0.1);
//    pnh.param("ikMaxItr", ikMaxItr, 10);
//    pnh.param("ikMaxTwist", ikMaxTwist, 0.1);
    //! @todo make this a command line parameter (needed to operate)
    nh.param("/robot_urdf", urdf, std::string());
    R2RosTrajectoryManager trajManager(ros::this_node::getName(), urdf, 1.0/rate, pnh);
    trajManager.setCartesianParameters(priorityLevels, linearTols, angularTols, momLimit, taskGain, ikMaxItr, ikMaxTwist);

    double x, y, z;
    std::string idMode;
    std::string baseFrame;
    nh.param("/gazebo/gravity_x", x, 0.0);
    nh.param("/gazebo/gravity_y", y, 0.0);
    nh.param("/gazebo/gravity_z", z, 0.0);
    pnh.param("idMode", idMode, std::string("full"));
    pnh.param("baseFrame", baseFrame, std::string("/r2/robot_world"));
    R2RosTreeId treeId(urdf, pnh);
    treeId.setGravity(x, y, z, baseFrame, idMode);

    R2RosArbiter arbiter(pnh);
        
    ros::Rate r(rate);
    
    while(nh.ok())
    {
        //! update hook
        trajManager.update();
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}
