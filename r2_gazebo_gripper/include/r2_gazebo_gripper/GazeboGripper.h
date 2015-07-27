#ifndef GAZEBOGRIPPER_H
#define GAZEBOGRIPPER_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo/physics/Gripper.hh>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Contact.hh>

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

        // Check contacts and handle appropriately
        void handleContacts(const std::vector<physics::Contact*>& contacts);

        // Attach linkPtr to the gripper
        void handleAttach(physics::LinkPtr linkPtr);

        // Detach an object from the gripper
        void handleDetach();

        // Pointer to the model
        physics::ModelPtr modelPtr;
        // This joint is used to attach the robot to the world
        physics::JointPtr fixedJointPtr;
        // Event callback for world updates.  Used to update contacts
        event::ConnectionPtr worldUpdateEvent;
        // The collision objects in the robot checked for contact with the world
        std::vector<std::string> collisionNames;

        // If true, the grippers are attached
        bool attached;
        // The amount of time required at low relative speed before grippers are attached
        double attachWait;
        // The amount of time required at high relative speed before grippers are detached
        double detachWait;
        // The amount of time between checks for contacts.  Useful since world updates are very frequent
        double updateTime;
        // The maximum relative speed between the gripper and the world allowed for contact
        double maxRelativeMotionRate;
        // The name of the link that is attached to the world (may be different from the gripper)
        std::string gripperAttachLink;

        // The most recent time that contacts were checked
        common::Time prevUpdateTime;
        // The amount of time the grippers have been in contact with the world
        double contactDuration;
        // The amount of time the grippers have not been in contact with the world
        double nonContactDuration;
        // The time of most recent contact
        common::Time prevContactUpdateTime;
        // The most recent difference in pose between the robot and the world. Used for speed computation
        math::Pose prevDiff;
        // The name of this plugin
        std::string pluginName;
    };
}

#endif
