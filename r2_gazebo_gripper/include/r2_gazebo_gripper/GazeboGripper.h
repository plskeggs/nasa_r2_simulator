#ifndef GAZEBOGRIPPER_H
#define GAZEBOGRIPPER_H

#include "common/Plugin.hh"
#include "common/Events.hh"

#include "physics/Gripper.hh"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "physics/Model.hh"
#include "physics/Joint.hh"
#include "physics/World.hh"
#include "physics/Link.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/Collision.hh"
#include "physics/Contact.hh"

namespace gazebo
{
    class GazeboGripper : public ModelPlugin
    {
    public:
        GazeboGripper();
        ~GazeboGripper();

        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void Init();

        // Called by the world update start event
        void onUpdate();

    private:
        // callback used when the gripper contacts an object
        void onContact(const std::string& _collisionName,
                       const physics::Contact& _contact);

        // check contact state and handle appropriately
        void handleContact();

        // Attach linkPtr to the gripper
        void handleAttach(physics::LinkPtr linkPtr);

        // Detach an object from the gripper
        void handleDetach();

        // Pointer to the model
        physics::ModelPtr modelPtr;
        physics::JointPtr fixedJointPtr;

        std::vector<physics::JointPtr> jointPtrs;
        std::vector<physics::LinkPtr> linkPtrs;
        std::vector<event::ConnectionPtr> connectionPtrs;

        std::map<std::string, physics::CollisionPtr> collisionPtrs;
        std::vector<physics::Contact> contacts;

        bool attached;
        double attachWait;
        double detachWait;
        double updateTime;
        double maxRelativeMotionRate;

        std::string gripperAttachLink;

        common::Time prevUpdateTime;
        double contactDuration;
        double nonContactDuration;
        common::Time prevContactUpdateTime;
        math::Pose prevDiff;
    };
}

#endif
