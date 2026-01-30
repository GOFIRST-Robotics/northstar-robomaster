#ifndef USING_CHASSIS_HPP_
#define USING_CHASSIS_HPP_

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "control/governor/fired_recently_governor.hpp"
#include "control/governor/plate_hit_governor.hpp"
#include "robot/testbed/test_def.hpp"

using namespace src::control::governor;
using namespace tap::control;
using namespace src::control::turret;
using namespace src::chassis;

#ifdef USING_CHASSIS

FiredRecentlyGovernor firedRecentlyGovernor(drivers(), 5000);

PlateHitGovernor plateHitGovernor(drivers(), 5000);

// GovernorWithFallbackCommand<2> beyBladeSlowOutOfCombat(
//     {&chassisSubsystem},
//     chassisBeyBladeSlowCommand,
//     chassisBeyBladeFastCommand,
//     {&firedRecentlyGovernor, &plateHitGovernor},
//     true);

// chassis Mappings
// ToggleCommandMapping bPressed(
//     drivers(),
//     {&beyBladeSlowOutOfCombat},
//     RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::B})));

// imu::ImuCalibrateCommand imuCalibrateCommand(
//     drivers(),
//     {{
//         &turret,
//         &chassisFrameYawTurretController,
//         &chassisFramePitchTurretController,
//         true,
//     }},
//     &chassisSubsystem);

tap::motor::DjiMotor yawMotor(
    drivers(),
    YAW_MOTOR_ID,
    CAN_BUS_MOTORS,
    true,
    "YawMotor",
    false,
    1,
    YAW_MOTOR_CONFIG.startEncoderValue);

ChassisSubsystem chassisSubsystem(
    drivers(),
    src::chassis::ChassisConfig{
        .leftFrontId = src::chassis::LEFT_FRONT_MOTOR_ID,
        .leftBackId = src::chassis::LEFT_BACK_MOTOR_ID,
        .rightBackId = src::chassis::RIGHT_BACK_MOTOR_ID,
        .rightFrontId = src::chassis::RIGHT_FRONT_MOTOR_ID,
        .canBus = CanBus::CAN_BUS1,
        .wheelVelocityPidConfig = modm::Pid<float>::Parameter(
            src::chassis::VELOCITY_PID_KP,
            src::chassis::VELOCITY_PID_KI,
            src::chassis::VELOCITY_PID_KD,
            src::chassis::VELOCITY_PID_MAX_ERROR_SUM),
    },
    &drivers()->turretMCBCanCommBus2,
    &yawMotor);

ChassisDriveCommand chassisDriveCommand(&chassisSubsystem, &drivers()->controlOperatorInterface);

ChassisOrientDriveCommand chassisOrientDriveCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface);

ChassisBeybladeCommand chassisBeyBladeSlowCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    1,
    -1,
    1,
    true);

ChassisBeybladeCommand chassisBeyBladeFastCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    1,
    -1,
    M_TWOPI,
    true);

// chassis Mappings
ToggleCommandMapping bPressed(
    drivers(),
    {&chassisBeyBladeFastCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::B})));

#endif

#endif  // USING_CHASSIS_HPP_