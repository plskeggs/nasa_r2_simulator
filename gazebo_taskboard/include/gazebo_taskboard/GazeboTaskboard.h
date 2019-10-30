/*
 * Copyright (C) 2013 TopCoder Inc., All Rights Reserved.
 */

/**
 * @file GazeboTaskboard.h
 * @brief This header file contains definition of GazeboTaskboardSlot1 class.
 * @author KennyAlive
 * @version 1.0
 */

#ifndef GAZEBOTASKBOARD_H
#define GAZEBOTASKBOARD_H

#include <boost/scoped_ptr.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

#include <gazebo_taskboard/ManipulatePowerCover.h>
#include <gazebo_taskboard/ManipulatePowerSwitch.h>
#include <gazebo_taskboard/ManipulateRockerSwitch.h>
#include <gazebo_taskboard/ManipulateNumPad.h>
#include <gazebo_taskboard/ManipulateSafeToggle.h>

#include <gazebo_taskboard/TaskboardPanelA.h>

namespace gazebo
{
   /**
    * @brief The GazeboTaskboardSlot1 class represents a controller for taskboard model (slot1).
    *
    * It is implemented as gazebo ModelPlugin, the instane of this class is associated with
    * taskboard model by the ROS gazebo plugin. The controller is responsible for simulating taskboard
    * behavior in reaction to external influences in similar way as it happens for real ISS taskboard.
    *
    * @author KennyAlive
    * @version 1.0
    */
    class GazeboTaskboardSlot1 : public ModelPlugin
    {
    public:
        /// @brief Plugin's default constructor.
        GazeboTaskboardSlot1();
        /// @brief Plugin's destructor.
        ~GazeboTaskboardSlot1();

        /// @brief Initializes plugin by providing associated model.
        virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    private:
        /// @brief Switch state with 2 stable positions
        enum TwoWayToggleSwitchState
        {
            eTwoWayState_Down,
            eTwoWayState_Up
        };
        /// @brief Switch state with 3 stable positions
        enum ThreeWayToggleSwitchState
        {
            eThreeWayState_Center,
            eThreeWayState_Down,
            eThreeWayState_Up
        };

    private:
        /// @brief Initializes all links that are used by the plugin.
        bool InitLinks();
        /// @brief Initializes all joints that are used by the plugin.
        bool InitJoints();
        /// @brief Derives current state from the model's positional configuration.
        void DeriveStateFromModel();

        /// @brief Simulates expected behavior according to the current state.
        void OnUpdate();

        /// @brief Handles power cover state changes.
        void MonitorPowerCoverStateChanges();
        /// @brief Handles power switch state changes.
        void MonitorPowerSwitchStateChanges();
        /// @brief Handles rocker switch A01 state changes.
        void MonitorRockerSwitchA01StateChanges();
        /// @brief Handles numpad buttons state changes.
        void MonitorNumpadStateChanges();
        /// @brief Handles safe toggles (A03/A04/A05) state changes.
        void MonitorSafeTogglesStateChanges();

        /// @brief Sends TaskboardPanelA message to /taskboard/TaskboardPanelA topic.
        void PublishState();

        struct Led;
        /// @brief Turns on/off the given LED.
        void SetLedState(Led& led, bool on);
        /// @brief Turns on all LEDs that should be on in the current state.
        void TurnOnLeds();
        /// @brief Turns off all LEDs.
        void TurnOffAllLeds();

        /// @brief Peforms power cover manipulation.
        bool ManipulatePowerCover(gazebo_taskboard::ManipulatePowerCover::Request& request,
                                  gazebo_taskboard::ManipulatePowerCover::Response& response);
        /// @brief Peforms power switch manipulation.
        bool ManipulatePowerSwitch(gazebo_taskboard::ManipulatePowerSwitch::Request& request,
                                   gazebo_taskboard::ManipulatePowerSwitch::Response& response);
        /// @brief Peforms rocker switch manipulation.
        bool ManipulateRockerSwitch(gazebo_taskboard::ManipulateRockerSwitch::Request& request,
                                    gazebo_taskboard::ManipulateRockerSwitch::Response& response);
        /// @brief Peforms numpad buttons manipulation.
        bool ManipulateNumPad(gazebo_taskboard::ManipulateNumPad::Request& request,
                              gazebo_taskboard::ManipulateNumPad::Response& response);
        /// @brief Peforms safe toggle (A03/A04/A05) manipulation.
        bool ManipulateSafeToggle(gazebo_taskboard::ManipulateSafeToggle::Request& request,
                                  gazebo_taskboard::ManipulateSafeToggle::Response& response);

        /// @brief Handles current manipulation state.
        void HandleManipulation();

        /// @brief Calculates empirical torque vector used to simulate switches/toggles rotation.
        ignition::math::Vector3<double> computeEmpiricalTorque(double deviationAngle, double initialValue,
                                             double snapCoeff, double snapExp, double torqueCoeff) const;

        /// @brief Makes joint transition from current OUT position to nearest UP/DOWN state.
        bool UpdateTransitionFromOutState2Way(physics::LinkPtr link, physics::JointPtr joint,
                                              TwoWayToggleSwitchState& state) const;
        /// @brief Makes joint transition from current OUT position to nearest UP/CENTER/DOWN state.
        bool UpdateTransitionFromOutState3Way(physics::LinkPtr link, physics::JointPtr joint,
                                              ThreeWayToggleSwitchState& state) const;

        /// @brief Gets current simulation time in seconds.
        double GetTime() const;

    private:
        /**
         * @brief first frame update flag.
         *
         * In Load method the physics engine is not ready for usage, for example,
         * we can not create model at that point. On the first update frame the physics
         * is ready and we can create all the necessary models (LEDs in our case).
         */
        bool firstFrameInitializationDone;

        /// @brief indicates if the leds setup is done
        bool ledsReady;

        /// @brief the handle of plugin node
        ros::NodeHandle node;

        /// @brief the model associated with this plugin
        physics::ModelPtr model;

        /// @brief pointer to the update event connection
        event::ConnectionPtr updateConnection;

        struct TaskboardSlot1State;
        /// @brief Taskboard Slot1 state
        boost::scoped_ptr<TaskboardSlot1State> state;

        struct TaskboardLeds;
        /// @brief the LEDs
        boost::scoped_ptr<TaskboardLeds> leds;

        // Links
        /// @brief power cover link
        physics::LinkPtr linkPowerCover;
        /// @brief power switch link
        physics::LinkPtr linkPowerSwitch;
        /// @brief rocker switch link
        physics::LinkPtr linkA01Switch;

        /// @brief the toggles count constant
        static const int SAFE_TOGGLES_COUNT = 3;
        /// @brief parent links for safe toggles
        physics::LinkPtr linksBaseSafeToggle[SAFE_TOGGLES_COUNT];
        /// @brief safe toggles links (A03/A04/A05)
        physics::LinkPtr linksSafeToggle[SAFE_TOGGLES_COUNT];
        /// @brief safe toggles revolute joints
        physics::JointPtr safeTogglesRevoluteJoints[SAFE_TOGGLES_COUNT];

        /// @brief the numpad buttons count constant
        static const int NUM_PAD_BUTTONS_COUNT = 9;
        /// @brief the numpad buttons links
        physics::LinkPtr linksNumPad[NUM_PAD_BUTTONS_COUNT];

        /// @brief publishers for sending messages about state changes
        ros::Publisher publisher;

        // Services
        /// @brief service for power cover manipulation
        ros::ServiceServer srv_manipulatePowerCover;
        /// @brief service for power switch manipulation
        ros::ServiceServer srv_manipulatePowerSwitch;
        /// @brief service for rocker switch manipulation
        ros::ServiceServer srv_manipulateRockerSwitch;
        /// @brief service for numpad buttons manipulation
        ros::ServiceServer srv_manipulateNumPad;
        /// @brief service for safe toggles manipulation
        ros::ServiceServer srv_manipulateSafeToggle;

        struct ManipulationState;
        /// @brief Holds entire manipulation state
        boost::scoped_ptr<ManipulationState> manipulationState;
    };

    // register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(GazeboTaskboardSlot1)
}

#endif // GAZEBOTASKBOARD_H
