#ifndef USING_FLYWHEEL_HPP_
#define USING_FLYWHEEL_HPP_

#include "tap/control/remote_map_state.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "robot/testbed/test_def.hpp"

using namespace tap::control;
using namespace src::control::flywheel;

#ifdef USING_FLYWHEEL
FlywheelSubsystem flywheel(drivers(), LEFT_MOTOR_ID, RIGHT_MOTOR_ID, UP_MOTOR_ID, CAN_BUS);

// flywheel commands
FlywheelRunCommand flywheelRunCommand(&flywheel);

// flywheel mappings
ToggleCommandMapping fPressed(
    drivers(),
    {&flywheelRunCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::F})));
#endif

#endif  // USING_FLYWHEEL_HPP_