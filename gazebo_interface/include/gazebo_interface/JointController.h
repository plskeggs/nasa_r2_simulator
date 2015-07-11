#ifndef JOINTCONTROLLER_H
#define JOINTCONTROLLER_H

#include <map>
#include <string>
#include <vector>

#include <common/PID.hh>
#include <common/Time.hh>
#include <physics/PhysicsTypes.hh>

#include <boost/thread.hpp>

#include "nasa_r2_common_msgs/JointStatus.h"
#include "nasa_r2_common_msgs/JointControl.h"

namespace gazebo
{
    class JointController
    {
    public:
        JointController(physics::JointPtr _jointPtr, bool _advancedMode = false);
        ~JointController();

        void setPosPid(double _p = 0.0, double _i = 0.0, double _d = 0.0,
                    double _imax = 0.0, double _imin = 0.0,
                    double _cmdMax = 0.0, double _cmdMin = 0.0);
        void setVelPid(double _p = 0.0, double _i = 0.0, double _d = 0.0,
                    double _imax = 0.0, double _imin = 0.0,
                    double _cmdMax = 0.0, double _cmdMin = 0.0);

        // set targets
        void setPosTarget(double target);
        void setVelTarget(double target);
        void setEffortTarget(double target);

        inline double getPosTarget() {return position;}
        inline double getVelTarget() {return velocity;}
        inline double getEffortTarget() {return effort;}

        inline void getJointLimits(double& lowLimit, double& highLimit) {lowLimit = jointLowLimit; highLimit = jointHighLimit;}

        // update PIDs and send forces to joints
        // stepTime is elapsed time since last update
        void update(common::Time& stepTime);

        // joint state management
        void setJointControl(const nasa_r2_common_msgs::JointControl::ConstPtr& msg);
        const nasa_r2_common_msgs::JointStatus& getJointStatus() const;

        void releaseBrake(bool release = true);
        void clearFaults();

    private:
        physics::JointPtr jointPtr;

        common::PID posPid;
        common::PID velPid;

        double position;
        double velocity;
        double effort;

        boost::mutex controllerMutex;

        bool advancedMode; // use state management stuff below

        // joint state management
        enum JointControlMode {POS_COM = 0, TORQ_COM = 1, IMP_COM = 2, VEL_COM = 3};
        enum JointFault {OK = 0};

        nasa_r2_common_msgs::JointStatusPtr currStatusPtr;
        JointControlMode controlMode;
        JointFault fault;

        double jointLowLimit;
        double jointHighLimit;
    };

    typedef boost::shared_ptr<JointController> JointControllerPtr;
}
#endif
