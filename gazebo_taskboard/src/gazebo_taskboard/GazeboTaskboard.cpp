/*
 * Copyright (C) 2013 TopCoder Inc., All Rights Reserved.
 */

/**
 * @file GazeboTaskboard.cpp
 * @brief This module implements GazeboTaskboardSlot1 class which represents gazebo ModelPlugin.
 * @author KennyAlive
 * @version 1.0
 */

#include <gazebo_taskboard/GazeboTaskboard.h>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <stdio.h>
#include <math.h>
#include <gazebo/common/Events.hh>
#include <std_msgs/String.h>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/msgs/MessageTypes.hh>

using namespace gazebo;
using namespace gazebo_taskboard;

/// @brief the middle position for power cover
const double POWER_COVER_JOINT_THRESHOLD = M_PI/4;

/// @brief the material for led in OFF state
const std::string LED_MATERIAL_OFF = "Gazebo/Grey";
/// @brief the material for green led
const std::string LED_MATERIAL_GREEN = "Gazebo/GreenGlow";
/// @brief the material for blue led
const std::string LED_MATERIAL_BLUE = "Gazebo/BlueGlow";

/// @brief led's model cylinder length used for toggles/switches
const double SIMPLE_LED_LENGTH = 0.0004;
/// @brief led's model cylinder radius used for toggles/switches
const double SIMPLE_LED_RADIUS = 0.003;
/// @brief led's model cylinder length used for numpad buttons
const double NUMPAD_LED_LENGTH = 0.0004;
/// @brief led's model cylinder radius used for numpad buttons
const double NUMPAD_LED_RADIUS = 0.0095;

/// @brief z coordinate offset for hiding led model in ON state
const double LED_HIDE_OFFSET = 100.0;
/// @brief the z coordinate offset for hiding led model in OFF state
const double LED_HIDE_OFFSET2 = 105.0;

/// @brief upper limit for A03/A04/A05 toggles as defined in URDF
const double SAFE_TOGGLE_UPPER_LIMIT_ANGLE = 0.0;
/// @brief lower limit for A03/A04/A05 toggles as defined in URDF
const double SAFE_TOGGLE_LOWER_LIMIT_ANGLE = -0.94;
/// @brief middle position for A03/A04/A05 toggles
const double SAFE_TOGGLE_MIDDLE_LIMIT_ANGLE = -0.47;
/// @brief half angle angle angle for A03/A04/A05 toggles
const double SAFE_TOGGLE_HALF_ANGLE = 0.47;

/// @brief Zero vector constant for convenience
const ignition::math::Vector3<double> ZERO_VECTOR = ignition::math::Vector3<double>(0, 0, 0);

/// @brief The available LED colors
enum LedColor
{
    eLedColor_Green, // Green led
    eLedColor_Blue   // Blue led
};

/**
 * @brief The Led structrure describes the LED in the simulator and holds current led state.
 *
 * The structure also holds some led configuration parameters since leds configuration is
 * a multi-step process (due to physics engine peculiarities).
 *
 * @author KennyAlive
 * @version 1.0
 */
struct GazeboTaskboardSlot1::Led
{
    /// @brief whether this LED is on/off
    bool isOn;

    // We need separate models for different states since ros gazebo plugin does
    // not provide a way to change model's material properties
    /// @brief model for ON state
    physics::ModelPtr onModel;
    /// @brief model for OFF state
    physics::ModelPtr offModel;

    // LED model and link names
    /// @brief model name for ON state
    std::string onModelName;
    /// @brief model name for OFF state
    std::string offModelName;
    /// @brief link name for ON state
    std::string onLinkName;
    /// @brief link name for OFF state
    std::string offLinkName;

    // LED model initialization parameters.
    // The LED model is initialized when gazebo world is fully loaded (first update frame)
    /// @brief the led index
    int index;
    /// @brief if the led is numpad button led
    bool numPadLed;
    /// @brief the color of this led
    LedColor color;
    /// @brief the pose of this led
    ignition::math::Pose3d pose;

    /// @brief Led default contructor
    Led();
    /// @brief Led constructor that initializes led instance with the given parameters
    Led(int index_, bool numPadLed_, LedColor color_, ignition::math::Pose3d pose_);
    /// @brief Create led model in the physics world
    void CreateModel(physics::WorldPtr world);
    /// @brief Setup led model in the physics world
    void SetupModel(physics::WorldPtr world);
};

/**
 * @brief This structure holds all led instances.
 *
 * Also TaskboardLeds provides leds convenient initialization methods that initialize all leds at one.
 *
 * @author KennyAlive
 * @version 1.0
 */
struct GazeboTaskboardSlot1::TaskboardLeds
{
    /// @brief the power switch led
    Led powerSwitchLed;
    /// @brief the rocker switch top led
    Led rockerSwitchUpLed;
    /// @brief the rocker switch bottom led
    Led rockerSwitchDownLed;
    /// @brief the numpad leds
    Led numPadLeds[NUM_PAD_BUTTONS_COUNT];
    /// @brief the A03 toggle led
    Led toggleA03Led;
    /// @brief the A04 toggle top led
    Led toggleA04TopLed;
    /// @brief the A04 toggle bottom led
    Led toggleA04BottomLed;
    /// @brief the A05 toggle led
    Led toggleA05Led;

    /// @brief TaskboardLeds  constructor.
    TaskboardLeds(const ignition::math::Pose3d& pose);

    /// @brief Creates led's models in the physics world.
    void CreateModels(physics::WorldPtr world);

    /// @brief Positions led's models in the world.
    void SetupModels(physics::WorldPtr world);
};

/**
 * @brief This structure holds state of the main taskboard elements.
 *
 * @author KennyAlive
 * @version 1.0
 */
struct GazeboTaskboardSlot1::TaskboardSlot1State
{
    /// @brief whether the beautiful red cover is open
    bool isCoverOpen;

    /// @brief power switch state (UP/DOWN)
    TwoWayToggleSwitchState powerSwitchState;

    /// @brief Rocker switch state (CENTER/UP/DOWN)
    ThreeWayToggleSwitchState rockerSwitchA01State;

    /// @brief numpad buttons SEL/UNSEL state
    bool numPadButtonsSelectedState[NUM_PAD_BUTTONS_COUNT];

    /// @brief whether the safe toggle in the OUT state
    bool isSafeToggleOut[SAFE_TOGGLES_COUNT];

    /// @brief A03 toggle state.
    /// If isSafeToggleOut[0] == true this is not used and state is defined as OUT
    TwoWayToggleSwitchState toggleA03State;

    /// @brief A04 toggle state.
    /// if isSafeToggleOut[1] == true this is not used and state is defined as OUT
    ThreeWayToggleSwitchState toggleA04State;

    /// @brief A05 toggle state.
    /// if isSafeToggleOut[2] == true this is not used and state is defined as OUT
    TwoWayToggleSwitchState toggleA05State;

    /**
     * @brief Initializes state to default values.
     */
    TaskboardSlot1State()
    {
        for (int i = 0; i < NUM_PAD_BUTTONS_COUNT; i++)
        {
            numPadButtonsSelectedState[i] = false;
        }
        for (int i = 0; i < SAFE_TOGGLES_COUNT; i++)
        {
            isSafeToggleOut[i] = false;
        }
        isCoverOpen = false;
        powerSwitchState = eTwoWayState_Down;
        rockerSwitchA01State = eThreeWayState_Center;
        toggleA03State = eTwoWayState_Down;
        toggleA04State = eThreeWayState_Center;
        toggleA05State = eTwoWayState_Down;
    }
};

/**
 * @brief Holds base manipulation parameters: startTime and duration.
 *
 * @author KennyAlive
 * @version 1.0
 */
struct BaseManipulationState
{
    /// @brief start time parameter
    double startTime;
    /// @brief duration parameter
    double duration;

    /// @brief Initializes parameters to zero values.
    BaseManipulationState()
    : startTime(0.0)
    , duration(0.0)
    {}
};
/// @brief Holds manipulation state for power cover.
struct PowerCoverManipulationState : BaseManipulationState
{
    /// @brief anglue velocity parameter
    ignition::math::Vector3<double> angularVelocity;
};
/// @brief Holds manipulation state for power switch.
struct PowerSwitchManipulationState : BaseManipulationState
{
    /// @brief anglue velocity parameter
    ignition::math::Vector3<double> angularVelocity;
};
/// @brief Holds manipulation state for rocker switch.
struct RockerSwitchManipulationState : BaseManipulationState
{
    /// @brief torque parameter
    ignition::math::Vector3<double> torque;
};
/// @brief Holds manipulation state for numpad buttons.
struct NumPadButtonManipulationState : BaseManipulationState
{
    /// @brief force parameter
    ignition::math::Vector3<double> force;
};
/// @brief Holds manipulation state for safe toggles (A03/A04/A05).
struct SafeToggleManipulationState : BaseManipulationState
{
    /// @brief generic value parameter
    double value;

    /// @brief Sets generic value to zero.
    SafeToggleManipulationState()
    : value(0.0)
    {}
};

/**
 * @brief Holds entire manipulation state and used to manipulate taskboard programatically.
 *
 * Manipulation is used for testing (simulatates robot actions) and when there is a
 * need to affect taskboard from the code.
 * Manipulation uses physics to change taskboard state (like applying force to the button
 * or torque to the toggle).
 *
 * @author KennyAlive
 * @version 1.0
 */
struct GazeboTaskboardSlot1::ManipulationState
{
    /// @brief Power cover manipulation state
    PowerCoverManipulationState powerCover;
    /// @brief Power switch manipulation state
    PowerSwitchManipulationState powerSwitch;
    /// @brief Rocker switch manipulation sate
    RockerSwitchManipulationState rockerSwitch;
    /// @brief Num Pad manipulation sate
    NumPadButtonManipulationState numPadButtons[NUM_PAD_BUTTONS_COUNT];
    /// @brief A03/A04/A05 safe toggles manipulation sate
    SafeToggleManipulationState safeToggles[SAFE_TOGGLES_COUNT][2];
};

/**
 * @brief Converts degrees to radians.
 *
 * @param degrees the angle value in degrees
 * @return the angle value in radians
 */
inline double deg2rad(double degrees)
{
    return M_PI / 180.0 * degrees;
}

//----------------------------------------------------------------------------------------
/**
 * @brief Plugin's default constructor.
 */
GazeboTaskboardSlot1::GazeboTaskboardSlot1()
: firstFrameInitializationDone(false)
, ledsReady(false)
, node("taskboard")
{}

/**
 * @brief Plugin's destructor.
 *
 * Stops listening to the world update event.
 */
GazeboTaskboardSlot1::~GazeboTaskboardSlot1()
{
    if (updateConnection)
    {
        // deprecated: event::Events::DisconnectWorldUpdateBegin(updateConnection);
        updateConnection.reset();
    }
}

/**
 * @brief Initializes plugin by providing associated model.
 *
 * @param _parent the models associated with this plugin instance
 * @param _sdf the SDF definition that holds models parameters
 */
void GazeboTaskboardSlot1::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    model = _parent;
    if (!model)
    {
        ROS_FATAL("No model is provided for GazeboTaskboard model plugin");
        return;
    }
    // Initialize links and joints from taskboard model
    if (!InitLinks() || !InitJoints()) // this also prints ROS_FATAL message in case of error
    {
        return;
    }

    // Listen to the update event.This event is broadcasted every simulation iteration.
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboTaskboardSlot1::OnUpdate, this));

    // Init services
    srv_manipulatePowerCover = node.advertiseService("manipulate_power_cover",
                                                     &GazeboTaskboardSlot1::ManipulatePowerCover, this);
    srv_manipulatePowerSwitch = node.advertiseService("manipulate_power_switch",
                                                     &GazeboTaskboardSlot1::ManipulatePowerSwitch, this);
    srv_manipulateRockerSwitch = node.advertiseService("manipulate_rocker_switch_a01",
                                                     &GazeboTaskboardSlot1::ManipulateRockerSwitch, this);
    srv_manipulateNumPad = node.advertiseService("manipulate_numpad",
                                                     &GazeboTaskboardSlot1::ManipulateNumPad, this);
    srv_manipulateSafeToggle = node.advertiseService("manipulate_safe_toggle",
                                                     &GazeboTaskboardSlot1::ManipulateSafeToggle, this);

    // Initialize publisher
    const int queueSize = 128;
    publisher = node.advertise<TaskboardPanelA>("TaskboardPanelA", queueSize);

    // Initialize manipulation state
    manipulationState.reset(new ManipulationState);

    //  Initialize LEDs
    leds.reset(new TaskboardLeds(model->WorldPose()));

    // Initialize controller according to the model state
    state.reset(new TaskboardSlot1State);
    DeriveStateFromModel();
}

/**
 * @brief Gets the model's link by the link name.
 *
 * The function also prints ROS fatal message if fails to get the link.
 *
 * @param model the model to get link from
 * @param linkName the link name
 *
 * @return the link object or null if the model does not have the requested link
 */
static physics::LinkPtr getLink(physics::ModelPtr model, const std::string& linkName)
{
    physics::LinkPtr link = model->GetLink(linkName);
    if (!link)
    {
        ROS_FATAL("Failed to get %s link from the model", linkName.c_str());
    }
    return link;
}

/**
 * @brief Initializes all links that are used by the plugin.
 *
 * @return true if the links are initialized successfully, otherwise false
 */
bool GazeboTaskboardSlot1::InitLinks()
{
    struct
    {
        physics::LinkPtr& linkRef;
        std::string linkName;
    } linksInfo[] =
    {
        { linkPowerCover        , "taskboard_slot1_switch1_cover"   },
        { linkPowerSwitch       , "taskboard_slot1_switch1"         },
        { linkA01Switch         , "taskboard_slot1_switch"          },
        { linksNumPad[0]        , "taskboard_slot1_A1"              },
        { linksNumPad[1]        , "taskboard_slot1_A2"              },
        { linksNumPad[2]        , "taskboard_slot1_A3"              },
        { linksNumPad[3]        , "taskboard_slot1_B1"              },
        { linksNumPad[4]        , "taskboard_slot1_B2"              },
        { linksNumPad[5]        , "taskboard_slot1_B3"              },
        { linksNumPad[6]        , "taskboard_slot1_C1"              },
        { linksNumPad[7]        , "taskboard_slot1_C2"              },
        { linksNumPad[8]        , "taskboard_slot1_C3"              },
        { linksBaseSafeToggle[0], "taskboard_slot1_switch2_pullout" },
        { linksBaseSafeToggle[1], "taskboard_slot1_switch3_pullout" },
        { linksBaseSafeToggle[2], "taskboard_slot1_switch4_pullout" },
        { linksSafeToggle[0]    , "taskboard_slot1_switch2"         },
        { linksSafeToggle[1]    , "taskboard_slot1_switch3"         },
        { linksSafeToggle[2]    , "taskboard_slot1_switch4"         }
    };
    const int numLinks = sizeof(linksInfo) / sizeof(linksInfo[0]);
    for (int i = 0; i < numLinks; i++)
    {
        linksInfo[i].linkRef = getLink(model, linksInfo[i].linkName);
        if (!linksInfo[i].linkRef)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief Gets the model's joint by the joint name.
 *
 * The function also prints ROS fatal message if fails to get the joint.
 *
 * @param model the model to get joint from
 * @param jointName the joint name
 *
 * @return the joint object or null if the model does not have the requested joint
 */
static physics::JointPtr getJoint(physics::ModelPtr model, const std::string& jointName)
{
    physics::JointPtr joint = model->GetJoint(jointName);
    if (!joint)
    {
        ROS_FATAL("Failed to get %s joint from the model", jointName.c_str());
    }
    return joint;
}

/**
 * @brief Initializes all joints that are used by the plugin.
 *
 * @return true if the joints are initialized successfully, otherwise false
 */
bool GazeboTaskboardSlot1::InitJoints()
{
    struct
    {
        physics::JointPtr& jointRef;
        std::string jointName;
    } jointsInfo[] =
    {
        { safeTogglesRevoluteJoints[0] , "taskboard_lower/taskboard_slot1_switch2" },
        { safeTogglesRevoluteJoints[1] , "taskboard_lower/taskboard_slot1_switch3" },
        { safeTogglesRevoluteJoints[2] , "taskboard_lower/taskboard_slot1_switch4" }
    };
    const int numJoints = sizeof(jointsInfo) / sizeof(jointsInfo[0]);
    for (int i = 0; i < numJoints; i++)
    {
        jointsInfo[i].jointRef = getJoint(model, jointsInfo[i].jointName);
        if (!jointsInfo[i].jointRef)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief Derives current state from the model's positional configuration.
 *
 * The state is derived only for elements that have more then one stable
 * position. For elements that have only single stable position (like rocker switch)
 * state is not derived from the model but directly initialized to single stable position.
 */
void GazeboTaskboardSlot1::DeriveStateFromModel()
{
    // Initialize powerCoverState
    ignition::math::Pose3d pose = linkPowerCover->RelativePose();
    double angle = pose.Rot().Roll();
    state->isCoverOpen = (angle < POWER_COVER_JOINT_THRESHOLD);

    // Initialize powerSwitchState.
    // If the power is on then turn on the leds that should be on in current state.
    pose = linkPowerSwitch->RelativePose();
    angle = pose.Rot().Roll();
    state->powerSwitchState = (angle < M_PI_2) ? eTwoWayState_Up : eTwoWayState_Down;

    // Initialize toggleA03State
    pose = linksSafeToggle[0]->RelativePose();
    angle = pose.Rot().Roll();
    state->toggleA03State = (angle < M_PI_2) ? eTwoWayState_Up : eTwoWayState_Down;

    // Initialize toggleA04State
    pose = linksSafeToggle[1]->RelativePose();
    angle = pose.Rot().Roll();
    if (angle < M_PI_2 - SAFE_TOGGLE_HALF_ANGLE/2)
    {
        state->toggleA04State = eThreeWayState_Up;
    }
    else if (angle < M_PI_2 + SAFE_TOGGLE_HALF_ANGLE/2)
    {
        state->toggleA04State = eThreeWayState_Center;
    }
    else
    {
        state->toggleA04State = eThreeWayState_Down;
    }

    // Initialize toggleA05State
    pose = linksSafeToggle[2]->RelativePose();
    angle = pose.Rot().Roll();
    state->toggleA05State = (angle < M_PI_2) ? eTwoWayState_Up : eTwoWayState_Down;
}

/**
 * @brief Simulates expected behavior according to the current state.
 *
 * The method is called regularly by Gazebo which allows the plugin
 * to function as an active object.
 */
void GazeboTaskboardSlot1::OnUpdate()
{
    // If condition is true then LED models are available and
    // we should set them up properly
    if (firstFrameInitializationDone && !ledsReady)
    {
        leds->SetupModels(model->GetWorld());
        TurnOnLeds(); // turn on necessary leds
        ledsReady = true;
        return;
    }
    // Create LED models on the first update frame.
    // This is necessary since physics engine is not ready when Load is called
    if (!firstFrameInitializationDone)
    {
        leds->CreateModels(model->GetWorld());
        firstFrameInitializationDone = true;
        return;
    }

    // Monitor state changes in taskboard elements and simulate expected behavior
    MonitorPowerCoverStateChanges();
    MonitorPowerSwitchStateChanges();
    MonitorRockerSwitchA01StateChanges();
    MonitorNumpadStateChanges();
    MonitorSafeTogglesStateChanges();

    // Process external manipulation
    HandleManipulation();

    // Spin the wheel! Make sure all topic messages are sent
    ros::spinOnce();
}

/**
 * @brief Handles power cover state changes.
 */
void GazeboTaskboardSlot1::MonitorPowerCoverStateChanges()
{
    ignition::math::Pose3d pose = linkPowerCover->RelativePose();
    const double angle = pose.Rot().Roll();
    const double threshold = deg2rad(1.0);

    if (angle > threshold && angle < M_PI_2 - threshold)
    {
        ignition::math::Vector3<double> torque = computeEmpiricalTorque(angle - POWER_COVER_JOINT_THRESHOLD, 1.0, 1.0, 1.5, 15);
        linkPowerCover->SetTorque(torque);
    }
    else if (angle <= threshold)
    {
        if (!state->isCoverOpen)
        {
            state->isCoverOpen = true;
            PublishState();
            // Stabilize link
            linkPowerCover->SetAngularVel(ZERO_VECTOR);
            linkPowerCover->SetTorque(ZERO_VECTOR);
        }
    }
    else if (angle >= M_PI_2 - threshold)
    {
        if (state->isCoverOpen)
        {
            state->isCoverOpen = false;
            PublishState();
            // Stabilize link
            linkPowerCover->SetAngularVel(ZERO_VECTOR);
            linkPowerCover->SetTorque(ZERO_VECTOR);
        }
    }
}

/**
 * @brief Handles power switch state changes.
 */
void GazeboTaskboardSlot1::MonitorPowerSwitchStateChanges()
{
    const double halfAngle = 0.419; // According to URDF
    const double upDirLimit = M_PI/2 - halfAngle;
    const double downDirLimit = M_PI/2 + halfAngle;
    const double threshold = deg2rad(1.0);

    ignition::math::Pose3d pose = linkPowerSwitch->RelativePose();
    const double angle = pose.Rot().Roll();

    if (angle > upDirLimit + threshold && angle < downDirLimit - threshold)
    {
        ignition::math::Vector3<double> torque = computeEmpiricalTorque(angle - M_PI_2, 1.0, 1.0, 1.3, 20);
        linkPowerSwitch->SetTorque(torque);
    }
    else if (angle <= upDirLimit + threshold)
    {
        if (state->powerSwitchState == eTwoWayState_Down)
        {
            state->powerSwitchState = eTwoWayState_Up;
            TurnOnLeds();
            PublishState();
            // Stabilize link
            linkPowerSwitch->SetAngularVel(ZERO_VECTOR);
            linkPowerSwitch->SetTorque(ZERO_VECTOR);
        }
    }
    else if (angle >= downDirLimit - threshold)
    {
        if (state->powerSwitchState == eTwoWayState_Up)
        {
            state->powerSwitchState = eTwoWayState_Down;
            TurnOffAllLeds();
            PublishState();
            // Stabilize link
            linkPowerSwitch->SetAngularVel(ZERO_VECTOR);
            linkPowerSwitch->SetTorque(ZERO_VECTOR);
        }
    }
}

/**
 * @brief Handles rocker switch A01 state changes.
 */
void GazeboTaskboardSlot1::MonitorRockerSwitchA01StateChanges()
{
    const double halfAngle = 0.5; // According to URDF
    const double upStateLimit = M_PI_2 - halfAngle;
    const double downStateLimit = M_PI_2 + halfAngle;
    const double threshold = deg2rad(2.0);

    ignition::math::Pose3d pose = linkA01Switch->RelativePose();
    double angle = pose.Rot().Roll();

    if (angle < M_PI_2 - threshold ||  angle > M_PI_2 + threshold)
    {
        // Switch is out of stable position, try to put it back
        const double sign = (angle < M_PI_2) ? 1.0 : -1.0;
        const double deviationAngle = sign * std::max(halfAngle - fabs(angle - M_PI_2), 0.0);
        ignition::math::Vector3<double> torque = computeEmpiricalTorque(deviationAngle, 8.5, 4.0, 2.5, 7);
        linkA01Switch->SetTorque(torque);

        // Check for new state (UP or DOWN)
        if (state->rockerSwitchA01State == eThreeWayState_Center)
        {
            if (angle < upStateLimit + threshold)
            {
                state->rockerSwitchA01State = eThreeWayState_Up;
                SetLedState(leds->rockerSwitchUpLed, true);
                PublishState();
            }
            else if (angle > downStateLimit - threshold)
            {
                state->rockerSwitchA01State = eThreeWayState_Down;
                SetLedState(leds->rockerSwitchDownLed, true);
                PublishState();
            }
        }
    }
    else
    {
        // Check if switch just returned to center position
        if (state->rockerSwitchA01State != eThreeWayState_Center)
        {
            if (leds->rockerSwitchUpLed.isOn)
            {
                SetLedState(leds->rockerSwitchUpLed, false);
            }
            else if (leds->rockerSwitchDownLed.isOn)
            {
                SetLedState(leds->rockerSwitchDownLed, false);
            }
            linkA01Switch->SetAngularVel(ZERO_VECTOR); // Stabilize switch at center position
            state->rockerSwitchA01State = eThreeWayState_Center;
            PublishState();
        }
    }
}

/**
 * @brief Handles numpad buttons state changes.
 */
void GazeboTaskboardSlot1::MonitorNumpadStateChanges()
{
    const double selRelOffset = -0.4811;
    const double epsilon = 0.0003;

    // Simulate springs which influence the buttons, so when we release
    // the button it returns to UNSEL position
    for (int i = 0; i < NUM_PAD_BUTTONS_COUNT; i++)
    {
        linksNumPad[i]->SetForce(ZERO_VECTOR);
        linksNumPad[i]->AddRelativeForce(ignition::math::Vector3<double>(0, 0, 5));
    }
    // Check for state changes
    for (int i = 0; i < NUM_PAD_BUTTONS_COUNT; i++)
    {
        const double offset = linksNumPad[i]->RelativePose().Pos().Y();

        bool selected = (offset < selRelOffset + epsilon);
        if (selected ^ state->numPadButtonsSelectedState[i])
        {
            state->numPadButtonsSelectedState[i] = selected;
            if (selected)
            {
                SetLedState(leds->numPadLeds[i], !leds->numPadLeds[i].isOn);
            }
            PublishState();
        }
    }
}

/**
 * @brief Makes joint transition from current OUT position to nearest UP/DOWN state.
 *
 * @param link the link that is in transition state
 * @param joint the revolute joint of the link
 * @param state if the method detects that transition is over then final state
 *              is returned via this reference
 *
 * @return true if the transition is over, otherwise false
 */
bool GazeboTaskboardSlot1::UpdateTransitionFromOutState2Way(
    physics::LinkPtr link, physics::JointPtr joint, TwoWayToggleSwitchState& state) const
{
    const double halfAngle = 0.47; // according to joint specification
    const double threshold = deg2rad(1.0);
    const double upDirLimit = M_PI_2 - halfAngle;
    const double downDirLimit = M_PI_2 + halfAngle;

    ignition::math::Pose3d pose = link->RelativePose();
    const double angle = pose.Rot().Roll();

    if (angle > upDirLimit + threshold && angle < downDirLimit - threshold)
    {
        ignition::math::Vector3<double> torque = computeEmpiricalTorque(angle - M_PI_2, 0.8, 1.0, 1.5, 15);
        link->SetTorque(torque);
        return false;
    }
    if (angle <= upDirLimit + threshold)
    {
        state = eTwoWayState_Up;
        // Latch the joint
        joint->SetUpperLimit(0, SAFE_TOGGLE_LOWER_LIMIT_ANGLE + threshold);
        joint->SetLowerLimit(0, SAFE_TOGGLE_LOWER_LIMIT_ANGLE - threshold);
    }
    else
    {
        state = eTwoWayState_Down;
        // Latch the joint
        joint->SetUpperLimit(0, SAFE_TOGGLE_UPPER_LIMIT_ANGLE + threshold);
        joint->SetLowerLimit(0, SAFE_TOGGLE_UPPER_LIMIT_ANGLE - threshold);
    }
    return true;
}

/**
 * @brief Makes joint transition from current OUT position to nearest UP/CENTER/DOWN state.
 *
 * @param link the link that is in transition state
 * @param joint the revolute joint of the link
 * @param state if the method detects that transition is over then final state
 *              is returned via this reference
 *
 * @return true if the transition is over, otherwise false
 */
bool GazeboTaskboardSlot1::UpdateTransitionFromOutState3Way(
    physics::LinkPtr link, physics::JointPtr joint, ThreeWayToggleSwitchState& state) const
{
    const double halfAngle = 0.47;
    const double upDirLimit = M_PI/2 - halfAngle;
    const double downDirLimit = M_PI/2 + halfAngle;
    const double threshold = deg2rad(1.0);

    ignition::math::Pose3d pose = link->RelativePose();
    double angle = pose.Rot().Roll();

    if (angle > upDirLimit + threshold && angle < M_PI_2 - threshold)
    {
        ignition::math::Vector3<double> torque = computeEmpiricalTorque(angle - (M_PI_2 - halfAngle/2), 0.8, 1.0, 1.5, 15);
        link->SetTorque(torque);
        return false;
    }
    else if (angle > M_PI_2 + threshold && angle < downDirLimit - threshold)
    {
        ignition::math::Vector3<double> torque = computeEmpiricalTorque(angle - (M_PI_2 + halfAngle/2), 0.8, 1.0, 1.5, 15);
        link->SetTorque(torque);
        return false;
    }
    if (angle <= upDirLimit + threshold)
    {
        state = eThreeWayState_Up;
        // Latch the joint
        joint->SetUpperLimit(0, SAFE_TOGGLE_LOWER_LIMIT_ANGLE + threshold);
        joint->SetLowerLimit(0, SAFE_TOGGLE_LOWER_LIMIT_ANGLE - threshold);
        link->SetTorque(ZERO_VECTOR);
    }
    else if (angle >= downDirLimit - threshold)
    {
        state = eThreeWayState_Down;
        // Latch the joint
        joint->SetUpperLimit(0, SAFE_TOGGLE_UPPER_LIMIT_ANGLE + threshold);
        joint->SetLowerLimit(0, SAFE_TOGGLE_UPPER_LIMIT_ANGLE - threshold);
        link->SetTorque(ZERO_VECTOR);
    }
    else
    {
        state = eThreeWayState_Center;
        // Latch the joint
        joint->SetUpperLimit(0, SAFE_TOGGLE_MIDDLE_LIMIT_ANGLE + threshold);
        joint->SetLowerLimit(0, SAFE_TOGGLE_MIDDLE_LIMIT_ANGLE - threshold);
        link->SetTorque(ZERO_VECTOR);
    }
    return true;
}

/**
 * @brief Handles safe toggles (A03/A04/A05) state changes.
 */
void GazeboTaskboardSlot1::MonitorSafeTogglesStateChanges()
{
    // The pull out distance as defined in URDF files.
    const double pullOutDistance = 0.002;
    // The delta below should be smaller then pull out offset.
    const double delta = 0.0004;

    // Check which joints is pulled out
    bool isOut[SAFE_TOGGLES_COUNT];
    float offsets[SAFE_TOGGLES_COUNT]; // toggles pulled out offsets
    for (int i = 0; i < SAFE_TOGGLES_COUNT; i++)
    {
        const ignition::math::Pose3d outPose = linksBaseSafeToggle[i]->RelativePose();
        const ignition::math::Pose3d pose = linksSafeToggle[i]->RelativePose();
        offsets[i] = std::max(pullOutDistance - outPose.CoordPositionSub(pose).Z(), 0.0);
        isOut[i] = (offsets[i] > pullOutDistance - delta);
    }
     // Set OUT state
    for (int i = 0; i < SAFE_TOGGLES_COUNT; i++)
    {
        if (!state->isSafeToggleOut[i] && isOut[i])
        {
            state->isSafeToggleOut[i] = true;

            // Release revolute joint
            safeTogglesRevoluteJoints[i]->SetUpperLimit(0, SAFE_TOGGLE_UPPER_LIMIT_ANGLE);
            safeTogglesRevoluteJoints[i]->SetLowerLimit(0, SAFE_TOGGLE_LOWER_LIMIT_ANGLE);

            // turn off LEDs
            switch (i)
            {
                case 0: // A03
                    SetLedState(leds->toggleA03Led, false);
                    break;
                case 1: // A04
                    SetLedState(leds->toggleA04TopLed, false);
                    SetLedState(leds->toggleA04BottomLed, false);
                    break;
                case 2: // A05
                    SetLedState(leds->toggleA05Led, false);
                    break;
            }
            PublishState();
        }
    }
    // Update two-way safe toggles that are in transition from OUT state
    if (state->isSafeToggleOut[0] && offsets[0] < pullOutDistance - 2*delta)
    {
        bool transitionDone = UpdateTransitionFromOutState2Way(
            linksSafeToggle[0], safeTogglesRevoluteJoints[0], state->toggleA03State);
        if (transitionDone)
        {
            state->isSafeToggleOut[0] = false;
            SetLedState(leds->toggleA03Led, state->toggleA03State == eTwoWayState_Up);
            PublishState();
        }
    }
    if (state->isSafeToggleOut[2] && offsets[2] < pullOutDistance - 2*delta)
    {
        bool transitionDone = UpdateTransitionFromOutState2Way(
            linksSafeToggle[2], safeTogglesRevoluteJoints[2], state->toggleA05State);
        if (transitionDone)
        {
            state->isSafeToggleOut[2] = false;
            SetLedState(leds->toggleA05Led, state->toggleA05State == eTwoWayState_Up);
            PublishState();
        }
    }
    // Update three-way safe toggles that are in transition from OUT state
    if (state->isSafeToggleOut[1] && offsets[1] < pullOutDistance - 2*delta)
    {
        bool transitionDone = UpdateTransitionFromOutState3Way(
            linksSafeToggle[1], safeTogglesRevoluteJoints[1], state->toggleA04State);
        if (transitionDone)
        {
            state->isSafeToggleOut[1] = false;
            SetLedState(leds->toggleA04TopLed, state->toggleA04State == eThreeWayState_Up);
            SetLedState(leds->toggleA04BottomLed, state->toggleA04State == eThreeWayState_Down);
            PublishState();
        }
    }
    // Simulate springs that pull the toggles in
    for (int i = 0; i < SAFE_TOGGLES_COUNT; i++)
    {
        if (state->isSafeToggleOut[i] || offsets[i] > delta)
        {
            linksSafeToggle[i]->SetForce(ZERO_VECTOR);
            linksSafeToggle[i]->AddRelativeForce(ignition::math::Vector3<double>(0, 0, -2));
        }
        else if (offsets[i] <= delta/2)
        {
            linksSafeToggle[i]->SetForce(ZERO_VECTOR);
            linksSafeToggle[i]->SetLinearVel(ZERO_VECTOR);
            linksSafeToggle[i]->SetAngularVel(ZERO_VECTOR);
        }
    }
}

/**
 * @brief Sends TaskboardPanelA message to /taskboard/TaskboardPanelA topic.
 *
 * The message is initialized with controller's current state.
 */
void GazeboTaskboardSlot1::PublishState()
{
    const std::string UP     = "UP";
    const std::string DOWN   = "DOWN";
    const std::string CENTER = "CENTER";
    const std::string ON     = "ON";
    const std::string OFF    = "OFF";
    const std::string SEL    = "SEL";
    const std::string UNSEL  = "UNSEL";
    const std::string OUT    = "OUT";

    // Initialize the message by collecting information from all elements
    TaskboardPanelA msg;

    msg.PANEL_POWER_COVER  = state->isCoverOpen ? UP : DOWN;
    msg.PANEL_POWER_SWITCH = state->powerSwitchState == eTwoWayState_Up ? UP : DOWN;
    msg.PANEL_POWER_LED    = leds->powerSwitchLed.isOn ? ON : OFF;

    msg.A01_ROCKER_SWITCH  = state->rockerSwitchA01State == eThreeWayState_Center
                              ? CENTER
                              : (state->rockerSwitchA01State == eThreeWayState_Up ? UP : DOWN);

    msg.A01_ROCKER_LED_TOP    = leds->rockerSwitchUpLed.isOn ? ON : OFF;
    msg.A01_ROCKER_LED_BOTTOM = leds->rockerSwitchDownLed.isOn ? ON : OFF;

    msg.A02_LED_NUM_PAD_A1 = leds->numPadLeds[0].isOn ? ON : OFF;
    msg.A02_LED_NUM_PAD_A2 = leds->numPadLeds[1].isOn ? ON : OFF;
    msg.A02_LED_NUM_PAD_A3 = leds->numPadLeds[2].isOn ? ON : OFF;
    msg.A02_LED_NUM_PAD_B1 = leds->numPadLeds[3].isOn ? ON : OFF;
    msg.A02_LED_NUM_PAD_B2 = leds->numPadLeds[4].isOn ? ON : OFF;
    msg.A02_LED_NUM_PAD_B3 = leds->numPadLeds[5].isOn ? ON : OFF;
    msg.A02_LED_NUM_PAD_C1 = leds->numPadLeds[6].isOn ? ON : OFF;
    msg.A02_LED_NUM_PAD_C2 = leds->numPadLeds[7].isOn ? ON : OFF;
    msg.A02_LED_NUM_PAD_C3 = leds->numPadLeds[8].isOn ? ON : OFF;

    msg.A02_NUM_PAD_A1 = state->numPadButtonsSelectedState[0] ? SEL : UNSEL;
    msg.A02_NUM_PAD_A2 = state->numPadButtonsSelectedState[1] ? SEL : UNSEL;
    msg.A02_NUM_PAD_A3 = state->numPadButtonsSelectedState[2] ? SEL : UNSEL;
    msg.A02_NUM_PAD_B1 = state->numPadButtonsSelectedState[3] ? SEL : UNSEL;
    msg.A02_NUM_PAD_B2 = state->numPadButtonsSelectedState[4] ? SEL : UNSEL;
    msg.A02_NUM_PAD_B3 = state->numPadButtonsSelectedState[5] ? SEL : UNSEL;
    msg.A02_NUM_PAD_C1 = state->numPadButtonsSelectedState[6] ? SEL : UNSEL;
    msg.A02_NUM_PAD_C2 = state->numPadButtonsSelectedState[7] ? SEL : UNSEL;
    msg.A02_NUM_PAD_C3 = state->numPadButtonsSelectedState[8] ? SEL : UNSEL;

    msg.A03_LED        = leds->toggleA03Led.isOn ? ON : OFF;
    msg.A04_LED_TOP    = leds->toggleA04TopLed.isOn ? ON : OFF;
    msg.A04_LED_BOTTOM = leds->toggleA04BottomLed.isOn ? ON : OFF;
    msg.A05_LED        = leds->toggleA05Led.isOn ? ON : OFF;

    msg.A03_TOGGLE = state->isSafeToggleOut[0]
                      ? OUT
                      : (state->toggleA03State == eTwoWayState_Up ? UP : DOWN);

    msg.A04_TOGGLE = state->isSafeToggleOut[1]
                      ? OUT
                      : (state->toggleA04State == eThreeWayState_Center
                          ? CENTER
                          : (state->toggleA04State == eThreeWayState_Up ? UP : DOWN));

    msg.A05_TOGGLE = state->isSafeToggleOut[2]
                      ? OUT
                      : (state->toggleA05State == eTwoWayState_Up ? UP : DOWN);

    // Finally publish the message
    publisher.publish(msg);
}

/**
 * @brief Turns on/off the given LED.
 *
 * @param led the Led instance
 * @param on true to turn on the LED, false to turn off the LED
 */
void GazeboTaskboardSlot1::SetLedState(Led& led, bool on)
{
    // Do nothing if the led is already in the requested state
    if (led.isOn == on)
    {
        return;
    }
    // Do not turn LED on if the power switch is off
    if (state->powerSwitchState != eTwoWayState_Up && on)
    {
        return;
    }

    // Get current pose
    ignition::math::Pose3d pose = led.onModel->RelativePose();
    ignition::math::Pose3d pose2 = led.offModel->RelativePose();

    // Update pose based on new LED state
    if (on)
    {
        pose.Pos().Z() -= LED_HIDE_OFFSET;
        pose2.Pos().Z() += LED_HIDE_OFFSET2;
    }
    else
    {
        pose.Pos().Z() += LED_HIDE_OFFSET;
        pose2.Pos().Z() -= LED_HIDE_OFFSET2;
    }
    led.onModel->SetLinkWorldPose(pose, led.onLinkName);
    led.offModel->SetLinkWorldPose(pose2, led.offLinkName);
    led.isOn = on;
}

/**
 * @brief Turns on all LEDs that should be on in the current state.
 */
void GazeboTaskboardSlot1::TurnOnLeds()
{
    if (state->powerSwitchState == eTwoWayState_Up)
    {
        // Power switch LED
        SetLedState(leds->powerSwitchLed, true);

        // Rocker switch LEDs
        if (state->rockerSwitchA01State == eThreeWayState_Up)
        {
            SetLedState(leds->rockerSwitchUpLed, true);
        }
        else if (state->rockerSwitchA01State == eThreeWayState_Down)
        {
            SetLedState(leds->rockerSwitchDownLed, true);
        }
        // A03 LED
        if (state->toggleA03State == eTwoWayState_Up)
        {
            SetLedState(leds->toggleA03Led, true);
        }
        // A05 LED
        if (state->toggleA05State == eTwoWayState_Up)
        {
            SetLedState(leds->toggleA05Led, true);
        }
        // A04 LEDs
        if (state->toggleA04State == eThreeWayState_Up)
        {
            SetLedState(leds->toggleA04TopLed, true);
        }
        else if (state->toggleA04State == eThreeWayState_Down)
        {
            SetLedState(leds->toggleA04BottomLed, true);
        }
    }
}

/**
 * @brief Turns off all LEDs.
 */
void GazeboTaskboardSlot1::TurnOffAllLeds()
{
    SetLedState(leds->powerSwitchLed, false);
    SetLedState(leds->rockerSwitchUpLed, false);
    SetLedState(leds->rockerSwitchDownLed, false);
    SetLedState(leds->toggleA03Led, false);
    SetLedState(leds->toggleA05Led, false);
    SetLedState(leds->toggleA04TopLed, false);
    SetLedState(leds->toggleA04BottomLed, false);

    for (int i = 0; i < NUM_PAD_BUTTONS_COUNT; i++)
    {
        SetLedState(leds->numPadLeds[i], false);
    }
}

//----------------------------------------------------------------------------------------
// Taskboard Manipulation
//----------------------------------------------------------------------------------------
/**
 * @brief Peforms power cover manipulation.
 *
 * Rotates the cover on the given angle.
 *
 * @param request the request object with manipulation parameters
 * @param response the response object
 *
 * @return true if the operation succeeded, otherwise false
 */
bool GazeboTaskboardSlot1::ManipulatePowerCover(
    ManipulatePowerCover::Request& request, ManipulatePowerCover::Response& response)
{
    ignition::math::Pose3d globalPose = model->WorldPose();
    const double velocity = request.angle / request.duration;

    PowerCoverManipulationState &manipState = manipulationState->powerCover;

    manipState.duration = request.duration;
    manipState.startTime = GetTime();
    manipState.angularVelocity = globalPose.Rot() * ignition::math::Vector3<double>(velocity, 0, 0);

    linkPowerCover->SetAngularVel(manipState.angularVelocity);
    return true;
}

/**
 * @brief Peforms power switch manipulation.
 *
 * Rotates the switch on the given angle.
 *
 * @param request the request object with manipulation parameters
 * @param response the response object
 *
 * @return true if the operation succeeded, otherwise false
 */
bool GazeboTaskboardSlot1::ManipulatePowerSwitch(
    ManipulatePowerSwitch::Request& request, ManipulatePowerSwitch::Response& response)
{
    ignition::math::Pose3d globalPose = model->WorldPose();
    const double velocity = request.angle / request.duration;

    PowerSwitchManipulationState &manipState = manipulationState->powerSwitch;

    manipState.duration = request.duration;
    manipState.startTime = GetTime();
    manipState.angularVelocity = globalPose.Rot() * ignition::math::Vector3<double>(velocity, 0, 0);

    linkPowerSwitch->SetAngularVel(manipState.angularVelocity);
    return true;
}

/**
 * @brief Peforms rocker switch manipulation.
 *
 * Applies the given amount of torque to the rocker switch.
 *
 * @param request the request object with manipulation parameters
 * @param response the response object
 *
 * @return true if the operation succeeded, otherwise false
 */
bool GazeboTaskboardSlot1::ManipulateRockerSwitch(
    ManipulateRockerSwitch::Request& request, ManipulateRockerSwitch::Response& response)
{
    ignition::math::Pose3d globalPose = model->WorldPose();

    RockerSwitchManipulationState &manipState = manipulationState->rockerSwitch;

    manipState.startTime = GetTime();
    manipState.duration = request.duration;
    manipState.torque = globalPose.Rot() * ignition::math::Vector3<double>(request.torque, 0, 0);

    linkA01Switch->SetTorque(manipState.torque);
    return true;
}

/**
 * @brief Peforms numpad buttons manipulation.
 *
 * Applies the given amount of force to the given button.
 *
 * @param request the request object with manipulation parameters
 * @param response the response object
 *
 * @return true if the operation succeeded, otherwise false
 */
bool GazeboTaskboardSlot1::ManipulateNumPad(
    ManipulateNumPad::Request& request, ManipulateNumPad::Response& response)
{
    if (request.index < 0 || request.index >= NUM_PAD_BUTTONS_COUNT)
    {
        return false;
    }
    NumPadButtonManipulationState &manipState = manipulationState->numPadButtons[request.index];

    manipState.startTime = GetTime();
    manipState.duration = request.duration;
    manipState.force = ignition::math::Vector3<double>(0, 0, -request.force);

    linksNumPad[request.index]->AddRelativeForce(manipState.force);
    return true;
}

/**
 * @brief Peforms safe toggle (A03/A04/A05) manipulation.
 *
 * Applies the given amount of force to the given toggle or rotates the
 * toggle on the given angle.
 *
 * @param request the request object with manipulation parameters
 * @param response the response object
 *
 * @return true if the operation succeeded, otherwise false
 */
bool GazeboTaskboardSlot1::ManipulateSafeToggle(
    ManipulateSafeToggle::Request& request, ManipulateSafeToggle::Response& response)
{
    if (request.index < 0 || request.index >= SAFE_TOGGLES_COUNT)
    {
        return false;
    }
    if (request.mode < 0 || request.mode >= 2)
    {
        return false;
    }
    SafeToggleManipulationState &manipState = manipulationState->safeToggles[request.index][request.mode];

    manipState.startTime = GetTime();
    manipState.duration = request.duration;
    manipState.value = request.value;

    physics::LinkPtr link = linksSafeToggle[request.index];
    if (request.mode == 0)
    {
        link->SetForce(ZERO_VECTOR);
        link->AddRelativeForce(ignition::math::Vector3<double>(0, 0, manipState.value));
    }
    else
    {
        const double velocity = manipState.value / manipState.duration;
        ignition::math::Pose3d basePose = model->WorldPose();
        ignition::math::Vector3<double> angularVelocity =  basePose.Rot() * ignition::math::Vector3<double>(velocity, 0, 0);
        link->SetAngularVel(angularVelocity);
    }
    return true;
}

/**
 * @brief Handles current manipulation state.
 */
void GazeboTaskboardSlot1::HandleManipulation()
{
    const double time = GetTime();
    // Check power cover
    if (manipulationState->powerCover.startTime)
    {
        const double elapsedTime = time - manipulationState->powerCover.startTime;
        if (elapsedTime >= manipulationState->powerCover.duration)
        {
            linkPowerCover->SetAngularVel(ZERO_VECTOR);
            manipulationState->powerCover.startTime = 0.0;
        }
        else
        {
            linkPowerCover->SetAngularVel(manipulationState->powerCover.angularVelocity);
        }
    }
    // Check power switch
    if (manipulationState->powerSwitch.startTime)
    {
        const double elapsedTime = time - manipulationState->powerSwitch.startTime;
        if (elapsedTime >= manipulationState->powerSwitch.duration)
        {
            linkPowerSwitch->SetAngularVel(ZERO_VECTOR);
            manipulationState->powerSwitch.startTime = 0.0;
        }
        else
        {
            linkPowerSwitch->SetAngularVel(manipulationState->powerSwitch.angularVelocity);
        }
    }
    // Check rocker switch A01
    if (manipulationState->rockerSwitch.startTime)
    {
        const double elapsedTime = time - manipulationState->rockerSwitch.startTime;
        if (elapsedTime >= manipulationState->rockerSwitch.duration)
        {
            linkA01Switch->SetTorque(ZERO_VECTOR);
            manipulationState->rockerSwitch.startTime = 0.0;
        }
        else
        {
            linkA01Switch->SetTorque(manipulationState->rockerSwitch.torque);
        }
    }
    // Check num pad buttons
    for (int i = 0; i < NUM_PAD_BUTTONS_COUNT; i++)
    {
        NumPadButtonManipulationState &manipState = manipulationState->numPadButtons[i];
        if (manipState.startTime)
        {
            const double elapsedTime = time - manipState.startTime;
            if (elapsedTime >= manipState.duration)
            {
                manipState.startTime = 0.0;
            }
            else
            {
                linksNumPad[i]->AddRelativeForce(manipState.force);
            }
        }
    }
    // Check safe toggles
    for (int i = 0; i < SAFE_TOGGLES_COUNT; i++)
    {
        SafeToggleManipulationState &manipState = manipulationState->safeToggles[i][0];
        if (manipState.startTime)
        {
            physics::LinkPtr link = linksSafeToggle[i];
            const double elapsedTime = time - manipState.startTime;
            if (elapsedTime >= manipState.duration) // reset manipulation influence
            {
                link->SetForce(ZERO_VECTOR);
                manipState.startTime = 0.0;
            }
            else // refresh manipulation influence
            {
                link->SetForce(ZERO_VECTOR);
                link->AddRelativeForce(ignition::math::Vector3<double>(0, 0,  manipState.value));
            }
        }
        SafeToggleManipulationState &manipState2 = manipulationState->safeToggles[i][1];
        if (manipState2.startTime)
        {
            physics::LinkPtr link = linksSafeToggle[i];
            const double elapsedTime = time - manipState2.startTime;
            if (elapsedTime >= manipState2.duration) // reset manipulation influence
            {
                link->SetAngularVel(ZERO_VECTOR);
                link->SetLinearVel(ZERO_VECTOR);
                manipState2.startTime = 0.0;
            }
            else // refresh manipulation influence
            {
                const double velocity = manipState2.value / manipState2.duration;
                ignition::math::Pose3d basePose = model->WorldPose();
                ignition::math::Vector3<double> angularVelocity =  basePose.Rot() * ignition::math::Vector3<double>(velocity, 0, 0);
                link->SetAngularVel(angularVelocity);
            }
        }
    }
}

/**
 * @brief Calculates empirical torque vector used to simulate switches/toggles rotation.
 *
 * @param deviationAngle the angle value counted from some central direction
 * @param initialValue specifies the initial amount of torque (this will be scaled by torqueCoeff)
 * @param snapCoeff scales the mantissa value
 * @param snapExp the exponent in the empirical expression
 * @param torqueCoeff the coefficient used to scale the entire torque vector
 *
 * @return the computed torque vector
 */
ignition::math::Vector3<double> GazeboTaskboardSlot1::computeEmpiricalTorque(
    double deviationAngle, double initialValue,
    double snapCoeff, double snapExp, double torqueCoeff) const
{
    // Horizontal direction along taskboard, it's an axis of rotation for the most switches.
    const ignition::math::Vector3<double> axis = model->WorldPose().Rot() * ignition::math::Vector3<double>(1, 0, 0);
    // Rotation direction
    const double sign = (deviationAngle < 0) ? 1.0 : -1.0;
    // Torque amount expression for physically plausible behavior
    const double torqueValue = (initialValue + pow(snapCoeff * fabs(deviationAngle), snapExp)) * sign * torqueCoeff;
    return axis * torqueValue;
}

/**
 * @brief Gets current simulation time in seconds.
 *
 * @return current simulation time in seconds
 */
double GazeboTaskboardSlot1::GetTime() const
{
    return model->GetWorld()->SimTime().Double();
}

//----------------------------------------------------------------------------------------
/**
 * @brief Led structure default constructor.
 */
GazeboTaskboardSlot1::Led::Led()
: isOn(false)
, index(-1)
, color(eLedColor_Green)
{}

/**
 * @brief Led structure constructor that initializes led instance with provided values.
 *
 * @param index_ the led index
 * @param numPadLed_ whether it's the led for numpad button or not
 * @param color_ the led color in ON state
 * @param pose_ the led position
 */
GazeboTaskboardSlot1::Led::Led(int index_, bool numPadLed_, LedColor color_, ignition::math::Pose3d pose_)
: isOn(false)
, index(index_)
, numPadLed(numPadLed_)
, color(color_)
, pose(pose_)
{}

/**
 * @brief Creates led's model in the physics world.
 *
 * @param world the physics world instance
 */
void GazeboTaskboardSlot1::Led::CreateModel(physics::WorldPtr world)
{
    const std::string sdfTemplate =
        "<gazebo version ='1.0'>\
            <model name ='MODELNAME' static='1'>\
                <origin pose='%1% %2% ZPOS %3% %4% %5%'/>\
                <link name ='LINKNAME' kinematic='1'>\
                    <collision name='COLLISIONNAME'>\
                        <geometry>GEOMETRY</geometry>\
                    </collision>\
                    <visual name='VISUALNAME'>\
                        <geometry>GEOMETRY</geometry>\
                        <material script='MATERIAL'/>\
                    </visual>\
                    <inertial mass='0.5'> \
                        <inertia ixx='1' ixy='0.0' ixz='0.0' iyy='1' iyz='0.0' izz='1'/>\
                    </inertial>\
                </link>\
            </model>\
        </gazebo>";

    // Figure out LED's material based on its color
    const std::string material = (color == eLedColor_Green) ? LED_MATERIAL_GREEN
                                                            : LED_MATERIAL_BLUE;
    // Create model and link names
    const std::string ledNameBase = boost::str(boost::format("taskboardLed%1%") % index);
    const std::string ledLinkNameBase = boost::str(boost::format("taskboardLedLink%1%") % index);
    const std::string ledCollisionNameBase = boost::str(boost::format("taskboardLedCollision%1%") % index);
    const std::string ledVisualNameBase = boost::str(boost::format("taskboardLedVisual%1%") % index);

    const std::string ledName = ledNameBase + "_ON";
    const std::string ledName2 = ledNameBase + "_OFF";
    const std::string ledLinkName = ledLinkNameBase + "_ON";
    const std::string ledLinkName2 = ledLinkNameBase + "_OFF";
    const std::string ledCollisionName = ledCollisionNameBase + "_ON";
    const std::string ledCollisionName2 = ledCollisionNameBase + "_OFF";
    const std::string ledVisualName = ledVisualNameBase + "_ON";
    const std::string ledVisualName2 = ledVisualNameBase + "_OFF";

    // Get LED model description depending on LED type
    std::string ledModel;
    ledModel = boost::str(boost::format("<cylinder length='%1%' radius='%2%'/>")
                              % (numPadLed ? NUMPAD_LED_LENGTH : SIMPLE_LED_LENGTH)
                              % (numPadLed ? NUMPAD_LED_RADIUS : SIMPLE_LED_RADIUS));

    // Prepare template
    double roll = pose.Rot().Roll();
    double pitch = pose.Rot().Pitch();
    double yaw = pose.Rot().Yaw();
    std::string xml = boost::str(boost::format(sdfTemplate)
                                 % pose.Pos().X() % pose.Pos().Y()
                                 % roll % pitch % yaw);
    std::string xml2 = xml;

    // SDF xml for turned on LED
    boost::replace_all(xml, "MODELNAME", ledName);
    boost::replace_all(xml, "ZPOS", boost::str(boost::format("%1%") % (pose.Pos().Z() + LED_HIDE_OFFSET)));
    boost::replace_all(xml, "LINKNAME", ledLinkName);
    boost::replace_all(xml, "COLLISIONNAME", ledCollisionName);
    boost::replace_all(xml, "VISUALNAME", ledVisualName);
    boost::replace_all(xml, "GEOMETRY", ledModel);
    boost::replace_all(xml, "MATERIAL", material);

    // SDF xml for turned off LED
    boost::replace_all(xml2, "MODELNAME", ledName2);
    boost::replace_all(xml2, "ZPOS", boost::str(boost::format("%1%") % (pose.Pos().Z() + LED_HIDE_OFFSET2)));
    boost::replace_all(xml2, "LINKNAME", ledLinkName2);
    boost::replace_all(xml2, "COLLISIONNAME", ledCollisionName2);
    boost::replace_all(xml2, "VISUALNAME", ledVisualName2);
    boost::replace_all(xml2, "GEOMETRY", ledModel);
    boost::replace_all(xml2, "MATERIAL", LED_MATERIAL_OFF);

    // Send request to physics engine to create LED models
    sdf::SDF ledSDF;
    ledSDF.SetFromString(xml);

    sdf::SDF ledSDF2;
    ledSDF2.SetFromString(xml2);

#if ROS_VERSION_MINIMUM(1, 9, 0)
    // ROS Groovy or above
    world->InsertModelSDF(ledSDF);
    world->InsertModelSDF(ledSDF2);
#else
    // ROS Fuerte
    world->InsertModel(ledSDF);
    world->InsertModel(ledSDF2);
#endif

    // Initialize led structure.
    // NOTE: at this moment we don't initialize led.onModel, led.offModel:
    // they will be available only after physics engine does next step
    onModelName = ledName;
    onLinkName = ledLinkName;
    offModelName = ledName2;
    offLinkName = ledLinkName2;
}

/**
 * @brief Initializes model references and position the led in the world.
 *
 * @param world the physics world instance
 */
void GazeboTaskboardSlot1::Led::SetupModel(physics::WorldPtr world)
{
    if (!onModel)
    {
        onModel = world->ModelByName(onModelName);
        if (!onModel)
        {
            ROS_FATAL("SetupModel: failed to get model %s", onModelName.c_str());
            return;
        }
        onModel->SetCollideMode("none");

        offModel = world->ModelByName(offModelName);
        if (!offModel)
        {
            ROS_FATAL("SetupModel: failed to get model %s", offModelName.c_str());
            return;
        }
        offModel->SetCollideMode("none");

        ignition::math::Pose3d pose2 = offModel->RelativePose();
        pose2.Pos().Z() -= LED_HIDE_OFFSET2;
        offModel->SetWorldPose(pose2);
    }
}

//----------------------------------------------------------------------------------------
/**
 * @brief TaskboardLeds constructor.
 *
 * Initializes all leds.
 *
 * @param modelPose the taskboard model pose
 */
GazeboTaskboardSlot1::TaskboardLeds::TaskboardLeds(const ignition::math::Pose3d& modelPose)
{
    struct LedsInfo {
        Led& led;
        bool numPadLed;
        LedColor color;
        ignition::math::Vector3<double> offset;
    } ledsInfo[] = {
        { powerSwitchLed,       false,  eLedColor_Green,  ignition::math::Vector3<double>(0.3250, -0.4720, 1.4110) },
        { rockerSwitchUpLed,    false,  eLedColor_Green,  ignition::math::Vector3<double>(0.3245, -0.4720, 1.3340) },
        { rockerSwitchDownLed,  false,  eLedColor_Blue,   ignition::math::Vector3<double>(0.3245, -0.4720, 1.2834) },
        { numPadLeds[0],        true,   eLedColor_Blue,   ignition::math::Vector3<double>(0.4496, -0.4755, 1.3358) },
        { numPadLeds[1],        true,   eLedColor_Green,  ignition::math::Vector3<double>(0.4230, -0.4755, 1.3358) },
        { numPadLeds[2],        true,   eLedColor_Blue,   ignition::math::Vector3<double>(0.3964, -0.4755, 1.3358) },
        { numPadLeds[3],        true,   eLedColor_Green,  ignition::math::Vector3<double>(0.4496, -0.4755, 1.3095) },
        { numPadLeds[4],        true,   eLedColor_Blue,   ignition::math::Vector3<double>(0.4230, -0.4755, 1.3095) },
        { numPadLeds[5],        true,   eLedColor_Green,  ignition::math::Vector3<double>(0.3964, -0.4755, 1.3095) },
        { numPadLeds[6],        true,   eLedColor_Blue,   ignition::math::Vector3<double>(0.4496, -0.4755, 1.2833) },
        { numPadLeds[7],        true,   eLedColor_Green,  ignition::math::Vector3<double>(0.4230, -0.4755, 1.2833) },
        { numPadLeds[8],        true,   eLedColor_Blue,   ignition::math::Vector3<double>(0.3964, -0.4755, 1.2833) },
        { toggleA03Led,         false,  eLedColor_Green,  ignition::math::Vector3<double>(0.4449, -0.4720, 1.2365) },
        { toggleA04TopLed,      false,  eLedColor_Green,  ignition::math::Vector3<double>(0.3812, -0.4720, 1.2365) },
        { toggleA04BottomLed,   false,  eLedColor_Blue,   ignition::math::Vector3<double>(0.3812, -0.4720, 1.1855) },
        { toggleA05Led,         false,  eLedColor_Green,  ignition::math::Vector3<double>(0.3185, -0.4720, 1.2365) }
    };
    const int ledsCount = sizeof(ledsInfo) / sizeof(ledsInfo[0]);
    const ignition::math::Quaternion<double> rot = ignition::math::Quaternion<double>(0.707, 0.707, 0, 0);

    for (int i = 0; i < ledsCount; i++)
    {
        const LedsInfo &info = ledsInfo[i];
        const ignition::math::Pose3d pose(modelPose.Pos() + modelPose.Rot() * info.offset, modelPose.Rot() * rot);
        info.led = Led(i, info.numPadLed, info.color, pose);
    }
}

/**
 * @brief Creates led's models in the physics world.
 *
 * Called on the first update frame. Does not work correctly if called from
 * plugin's Load method.
 *
 * @param world the physics world instance
 */
void GazeboTaskboardSlot1::TaskboardLeds::CreateModels(physics::WorldPtr world)
{
    for (int i = 0; i < NUM_PAD_BUTTONS_COUNT; i++)
    {
        numPadLeds[i].CreateModel(world);
    }
    powerSwitchLed.CreateModel(world);
    rockerSwitchUpLed.CreateModel(world);
    rockerSwitchDownLed.CreateModel(world);
    toggleA03Led.CreateModel(world);
    toggleA04TopLed.CreateModel(world);
    toggleA04BottomLed.CreateModel(world);
    toggleA05Led.CreateModel(world);
}

/**
 * @brief Positions led's models in the world.
 *
 * @param world the physics world instance
 */
void GazeboTaskboardSlot1::TaskboardLeds::SetupModels(physics::WorldPtr world)
{
    for (int i = 0; i < NUM_PAD_BUTTONS_COUNT; i++)
    {
        numPadLeds[i].SetupModel(world);
    }
    powerSwitchLed.SetupModel(world);
    rockerSwitchUpLed.SetupModel(world);
    rockerSwitchDownLed.SetupModel(world);
    toggleA03Led.SetupModel(world);
    toggleA04TopLed.SetupModel(world);
    toggleA04BottomLed.SetupModel(world);
    toggleA05Led.SetupModel(world);
}
