#include <r2_controllers_ros/R2RosTreeId.hpp>

R2RosTreeId::R2RosTreeId(std::string urdf, ros::NodeHandle &nh):
    gravityFrame("/r2/robot_world"),
    argos(false),
    useImu(false),
    gravityBase("/r2/robot_world"),
    idMode("full"),
    yankLimBFF(0.1),
    loopRate(0.02),
    baseFrame("/r2/robot_world"),
    bfCmd("/r2/robot_world"),
    bfFactor(1.0),
    newBase(false)
{
    id.loadFromFile(urdf);
    jd.InitializeMaps(id.getTree());

    torqueOut     = nh.advertise<sensor_msgs::JointState>("/joint_torque_commands", 1);
    inertiaOut    = nh.advertise<sensor_msgs::JointState>("/treeid_inertia", 1);
    segForceOut   = nh.advertise<nasa_r2_common_msgs::WrenchState>("/treeid_segment_forces", 1);
    completionOut = nh.advertise<nasa_r2_common_msgs::StringArray>("/completion", 1);

    baseFrameIn   = nh.subscribe("/base_frames", 1, &R2RosTreeId::baseFrameCallback, this);
    forceIn       = nh.subscribe("/treeid_external_forces", 1, &R2RosTreeId::forceCallback, this);
    jointStateIn  = nh.subscribe("/joint_states", 1, &R2RosTreeId::jointStateCallback, this);
    trajIn        = nh.subscribe("/joint_commands", 1, &R2RosTreeId::trajCallback, this);
    imuIn         = nh.subscribe("imu", 1, &R2RosTreeId::imuCallback, this);
}

bool R2RosTreeId::setGravity(double x, double y, double z, std::string frame, std::string mode)
{
    if(!id.isBaseFrameInTree(frame))
    {
        ROS_ERROR("Unable to setGravity in R2RosTreeId, no such frame");
        return false;
    }
    idMode = mode;
    gravityFrame = frame;
    gravity.resize(3);
    gravity[0] = x;
    gravity[1] = y;
    gravity[2] = z;
    baseMsg.data.resize(1);
    baseMsg.data[0] = frame;
    rmt.setBaseFrame(baseMsg, frame);
    completionMsg.header.stamp = ros::Time::now();
    completionMsg.data.clear();
    completionMsg.data.push_back("base_frame");
    completionOut.publish(completionMsg);
    return true;
}

void R2RosTreeId::baseFrameCallback(const nasa_r2_common_msgs::StringArray &msg)
{
    baseMsg = msg;
    if (rmt.setBaseFrame(baseMsg, bfCmd) > 0)
    {
        newBase = true;
        ROS_INFO("Received base frame");
    }
    else
    {
        ROS_ERROR("No base frames specified");
    }
}

void R2RosTreeId::forceCallback(const nasa_r2_common_msgs::WrenchState &msg)
{
    forceMsg = msg;
}

void R2RosTreeId::trajCallback(const sensor_msgs::JointState &msg)
{
    trajStateMsg = msg;
}

void R2RosTreeId::imuCallback(const sensor_msgs::Imu &msg)
{
    imuInMsg = msg;
}

void R2RosTreeId::jointStateCallback(const sensor_msgs::JointState &msg)
{
    if (idMode == nasa_r2_common_msgs::JointCommand::FULL || idMode == nasa_r2_common_msgs::JointCommand::INERTIA)
    {
        rmt.setJointData(msg, trajStateMsg, forceMsg, jd);
    }
    else
    {
        rmt.setJointData(msg, emptyTrajMsg, emptyForceMsg, jd);
    }

    if (idMode == nasa_r2_common_msgs::JointCommand::FULL || idMode == nasa_r2_common_msgs::JointCommand::GRAVITY)
    {
        // Blend base frame switching
        if (gravity[0] != 0 || gravity[1] != 0 || gravity[2] != 0)
        {
            if (baseFrame != bfCmd)
            {
                bfFactor -= yankLimBFF*loopRate;
            }

            if (baseFrame == bfCmd && bfFactor < 1.0)
            {
                bfFactor += yankLimBFF*loopRate;
            }

            if (bfFactor <= 0.0 )
            {
                baseFrame = bfCmd;
                bfFactor = 0.0;
            }

            if (bfFactor > 1.0)
            {
                bfFactor = 1.0;
            }
        }

        // Find gravity vector and do dynamics
        gravity_kdl = bfFactor * KDL::Vector(-gravity[0], -gravity[1], -gravity[2]);
        id.getAccelInBaseFrame(gravity_kdl, gravityFrame, jd, baseFrame, a_in);
        // Gravity Comp
        id.treeRecursiveNewtonEuler(jd, baseFrame, "null", v_in, a_in, f_out, I_out);
    }
    else
    {
        // no gravity comp
        id.treeRecursiveNewtonEuler(jd, baseFrame, "null", zeroTwist, zeroTwist, f_out, I_out);
    }

    // Populate & send messages
    if (idMode != nasa_r2_common_msgs::JointCommand::NONE)
    {
        rmt.getJointCommand(jd, jointTorqueMsg);
    }
    else
    {
        rmt.setEmptyTorqueMsg(jd, jointTorqueMsg);
    }

    // Inertia and force happens regardless
    rmt.getJointInertias(jd, jointInertiaMsg);
    rmt.getSegmentForces(jd, segForceMsg);

    torqueOut.publish(jointTorqueMsg);
    inertiaOut.publish(jointInertiaMsg);
    segForceOut.publish(segForceMsg);

    if (bfFactor >= 1.0 && bfCmd == baseFrame && newBase)
    {
        completionMsg.header.stamp = ros::Time::now();
        completionOut.publish(completionMsg);
        newBase = false;
    }
}

