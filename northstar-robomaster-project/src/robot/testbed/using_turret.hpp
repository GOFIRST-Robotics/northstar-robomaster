#ifndef USING_TURRET_HPP_
#define USING_TURRET_HPP_

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"

#include "robot/testbed/test_def.hpp"

using namespace tap::control;
using namespace src::control::turret;
using namespace tap::motor;

#if defined(USING_TURRET) && !defined(USING_REV)

// turret subsystem
tap::motor::DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    CAN_BUS_MOTORS,
    true,
    "PitchMotor",
    false,
    1,
    PITCH_MOTOR_CONFIG.startEncoderValue);

tap::motor::DjiMotor yawMotor(
    drivers(),
    YAW_MOTOR_ID,
    CAN_BUS_MOTORS,
    true,
    "YawMotor",
    false,
    1,
    YAW_MOTOR_CONFIG.startEncoderValue);

StandardTurretSubsystem turretSubsystem(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

// turret controlers
src::control::turret::algorithms::
    ChassisFramePitchTurretController chassisFramePitchTurretController(
        turretSubsystem.pitchMotor,
        chassis_rel::PITCH_PID_CONFIG);

src::control::turret::algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    turretSubsystem.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

src::control::turret::algorithms::
    WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
        *drivers(),
        turretSubsystem.yawMotor,
        world_rel_chassis_imu::YAW_PID_CONFIG);

src::control::turret::algorithms::
    WorldFramePitchChassisImuTurretController worldFramePitchChassisImuController(
        *drivers(),
        turretSubsystem.pitchMotor,
        world_rel_chassis_imu::PITCH_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretPosPid(world_rel_turret_imu::PITCH_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretVelPid(world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

// for imu can com giving imu data from turret to chassis
src::control::turret::algorithms::
    WorldFramePitchTurretCanImuCascadePidTurretController worldFramePitchTurretCanImuController(
        getTurretMCBCanComm(),
        turretSubsystem.pitchMotor,
        worldFramePitchTurretPosPid,
        worldFramePitchTurretVelPid);

src::control::turret::algorithms::
    WorldFrameYawTurretCanImuCascadePidTurretController worldFrameYawTurretCanImuController(
        getTurretMCBCanComm(),
        turretSubsystem.yawMotor,
        worldFrameYawTurretPosPid,
        worldFrameYawTurretVelPid);

// for imu fixed on turret
src::control::turret::algorithms::
    WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
        *drivers(),
        turretSubsystem.pitchMotor,
        worldFramePitchTurretPosPid,
        worldFramePitchTurretVelPid);

src::control::turret::algorithms::
    WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
        *drivers(),
        turretSubsystem.yawMotor,
        worldFrameYawTurretPosPid,
        worldFrameYawTurretVelPid);

// turret commands
user::TurretUserControlCommand turretUserControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turretSubsystem,
    &worldFrameYawTurretImuController,
    &worldFramePitchChassisImuController,  //&worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

cv::TurretCVControlCommand turretCVControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    drivers()->visionComms,
    &turretSubsystem,
    &worldFrameYawTurretImuController,
    &worldFramePitchChassisImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

user::TurretUserWorldRelativeCommand turretUserWorldRelativeCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turretSubsystem,
    &worldFrameYawChassisImuController,
    &worldFramePitchChassisImuController,
    &worldFrameYawTurretCanImuController,
    &worldFramePitchTurretCanImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

HoldCommandMapping xPressed(
    drivers(),
    {&turretCVControlCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::X})));

#endif

#if defined(USING_TURRET) && defined(USING_REV)

tap::motor::DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    CAN_BUS_MOTORS,
    true,
    "PitchMotor",
    false,
    1,
    PITCH_MOTOR_CONFIG.startEncoderValue);

tap::motor::RevMotor yawMotor1(
    drivers(),
    REV_MOTOR1,
    CanBus::CAN_BUS1,
    RevMotor::ControlMode::VOLTAGE,
    false,
    "YawMotor1",
    18.0f / 120.0f);  // gear ratio

tap::motor::RevMotor yawMotor2(
    drivers(),
    tap::motor::REV_MOTOR2,
    CanBus::CAN_BUS1,
    RevMotor::ControlMode::VOLTAGE,
    false,
    "YawMotor2",
    18.0f / 120.0f);  // gear ratio

RevTurretSubsystem revTurret(
    drivers(),
    &pitchMotor,
    &yawMotor1,
    &yawMotor2,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_REV_CONFIG);

algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
    *drivers(),
    revTurret.yawMotor,
    world_rel_chassis_imu::YAW_PID_CONFIG);

algorithms::WorldFramePitchChassisImuTurretController worldFramePitchChassisImuController(
    *drivers(),
    revTurret.pitchMotor,
    world_rel_chassis_imu::PITCH_PID_CONFIG);

user::NeoTurretUserControlCommand turretUserControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &revTurret,
    &worldFrameYawChassisImuController,
    &worldFramePitchChassisImuController,  //&worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

#endif

#endif  // USING_TURRET_HPP_