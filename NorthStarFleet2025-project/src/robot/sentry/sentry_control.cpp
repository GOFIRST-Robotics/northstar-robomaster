#ifdef TARGET_SENTRY

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "../../robot-type/robot_type.hpp"
#include "robot/sentry/sentry_drivers.hpp"

#include "drivers_singleton.hpp"

// chasis
#include "control/chassis/chassis_beyblade_command.hpp"
#include "control/chassis/chassis_drive_command.hpp"
#include "control/chassis/chassis_field_command.hpp"
#include "control/chassis/chassis_orient_drive_command.hpp"
#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/constants/chassis_constants.hpp"

// agitator
#include "control/agitator/constant_velocity_agitator_command.hpp"
#include "control/agitator/constants/agitator_constants.hpp"
#include "control/agitator/unjam_spoke_agitator_command.hpp"
#include "control/agitator/velocity_agitator_subsystem.hpp"

// turret
#include "control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "control/turret/constants/turret_constants.hpp"
// #include "control/turret/user/turret_quick_turn_command.hpp"
#include "robot/sentry/sentry_turret_subsystem.hpp"
#include "robot/sentry/sentry_turret_user_world_relative_command.hpp"

// flywheel
#include "control/flywheel/flywheel_constants.hpp"
#include "control/flywheel/flywheel_run_command.hpp"
#include "control/flywheel/flywheel_subsystem.hpp"

// imu
#include "robot/sentry/sentry_imu_calibrate_command.hpp"

// safe disconnect
#include "control/safe_disconnect.hpp"

using tap::can::CanBus;
using tap::communication::serial::Remote;
using tap::motor::MotorId;

using namespace tap::control::setpoint;
using namespace tap::control;
using namespace src::sentry;
using namespace src::control::turret;
using namespace src::control;
using namespace src::flywheel;
using namespace src::control::flywheel;
using namespace src::agitator;
using namespace src::control::agitator;

driversFunc drivers = DoNotUse_getDrivers;

namespace sentry_control
{
inline src::can::TurretMCBCanComm &getTurretMCBCanComm() { return drivers()->turretMCBCanCommBus2; }
// agitator subsystem
VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG);

// agitator commands
ConstantVelocityAgitatorCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

UnjamSpokeAgitatorCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

// agitator mappings
HoldRepeatCommandMapping leftMousePressed(
    drivers(),
    {&rotateAndUnjamAgitator},
    RemoteMapState(RemoteMapState::MouseButton::LEFT),
    false);

// turret subsystem
tap::motor::DjiMotor pitchMotorBottom(
    drivers(),
    PITCH_MOTOR_BOTTOM_ID,
    CAN_BUS_MOTORS,
    true,
    "Pitch Motor Bottom",
    false,
    1,
    PITCH_MOTOR_CONFIG_BOTTOM.startEncoderValue);

tap::motor::DjiMotor yawMotorBottom(
    drivers(),
    YAW_MOTOR_BOTTOM_ID,
    CAN_BUS_MOTORS,
    true,
    "Yaw Motor Bottom",
    false,
    1,
    YAW_MOTOR_CONFIG_BOTTOM.startEncoderValue);

tap::motor::DjiMotor pitchMotorTop(
    drivers(),
    PITCH_MOTOR_TOP_ID,
    CAN_BUS_MOTORS,
    true,
    "Pitch Motor Top",
    false,
    1,
    PITCH_MOTOR_CONFIG_TOP.startEncoderValue);

tap::motor::DjiMotor yawMotorTop(
    drivers(),
    YAW_MOTOR_TOP_ID,
    CAN_BUS_MOTORS,
    true,
    "Yaw Motor Top",
    false,
    1,
    YAW_MOTOR_CONFIG_TOP.startEncoderValue);

SentryTurretSubsystem sentryTurrets(
    drivers(),
    &pitchMotorBottom,
    &yawMotorBottom,
    &pitchMotorTop,
    &yawMotorTop,
    PITCH_MOTOR_CONFIG_BOTTOM,
    YAW_MOTOR_CONFIG_BOTTOM,
    PITCH_MOTOR_CONFIG_TOP,
    YAW_MOTOR_CONFIG_TOP,
    &getTurretMCBCanComm());

// turret controlers
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretControllerBottom(
    sentryTurrets.pitchMotorBottom,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFramePitchTurretController chassisFramePitchTurretControllerTop(
    sentryTurrets.pitchMotorTop,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretControllerBottom(
    sentryTurrets.yawMotorBottom,
    chassis_rel::YAW_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretControllerTop(
    sentryTurrets.yawMotorTop,
    chassis_rel::YAW_PID_CONFIG);

algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuControllerBottom(
    *drivers(),
    sentryTurrets.yawMotorBottom,
    world_rel_chassis_imu::YAW_PID_CONFIG);

algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuControllerTop(
    *drivers(),
    sentryTurrets.yawMotorTop,
    world_rel_chassis_imu::YAW_PID_CONFIG);

algorithms::WorldFramePitchChassisImuTurretController worldFramePitchChassisImuControllerBottom(
    *drivers(),
    sentryTurrets.pitchMotorBottom,
    world_rel_chassis_imu::PITCH_PID_CONFIG);

algorithms::WorldFramePitchChassisImuTurretController worldFramePitchChassisImuControllerTop(
    *drivers(),
    sentryTurrets.pitchMotorTop,
    world_rel_chassis_imu::PITCH_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretImuPosPidBottom(
    world_rel_turret_imu::PITCH_POS_PID_CONFIG_BOTTOM);

tap::algorithms::SmoothPid worldFramePitchTurretImuVelPidBottom(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG_BOTTOM);

algorithms::
    WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuControllerBottom(
        *drivers(),
        sentryTurrets.pitchMotorBottom,
        worldFramePitchTurretImuPosPidBottom,
        worldFramePitchTurretImuVelPidBottom);

tap::algorithms::SmoothPid worldFramePitchTurretImuPosPidTop(
    world_rel_turret_imu::PITCH_POS_PID_CONFIG_TOP);

tap::algorithms::SmoothPid worldFramePitchTurretImuVelPidTop(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG_TOP);

algorithms::
    WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuControllerTop(
        *drivers(),
        sentryTurrets.pitchMotorTop,
        worldFramePitchTurretImuPosPidTop,
        worldFramePitchTurretImuVelPidTop);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPidBottom(
    world_rel_turret_imu::YAW_POS_PID_CONFIG_BOTTOM);

tap::algorithms::SmoothPid worldFrameYawTurretImuVelPidBottom(
    world_rel_turret_imu::YAW_VEL_PID_CONFIG_BOTTOM);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuControllerBottom(
    *drivers(),
    sentryTurrets.yawMotorBottom,
    worldFrameYawTurretImuPosPidBottom,
    worldFrameYawTurretImuVelPidBottom);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPidTop(
    world_rel_turret_imu::YAW_POS_PID_CONFIG_TOP);

tap::algorithms::SmoothPid worldFrameYawTurretImuVelPidTop(
    world_rel_turret_imu::YAW_VEL_PID_CONFIG_TOP);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuControllerTop(
    *drivers(),
    sentryTurrets.yawMotorTop,
    worldFrameYawTurretImuPosPidTop,
    worldFrameYawTurretImuVelPidTop);

// turret commands
user::SentryTurretUserControlCommand turretWRChassisImuCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &sentryTurrets,
    &worldFrameYawTurretImuControllerBottom,
    &worldFramePitchChassisImuControllerBottom,
    &chassisFrameYawTurretControllerTop,  // controler for top turret
    &worldFramePitchChassisImuControllerTop,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR,
    M_TWOPI);  // +- offset max rads

user::SentryTurretUserWorldRelativeCommand turretsUserWorldRelativeCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &sentryTurrets,
    &worldFrameYawChassisImuControllerBottom,
    &worldFramePitchChassisImuControllerBottom,
    &worldFrameYawTurretImuControllerBottom,
    &worldFramePitchTurretImuControllerBottom,
    &chassisFrameYawTurretControllerTop,
    &worldFramePitchChassisImuControllerTop,
    &worldFramePitchTurretImuControllerTop,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR,
    M_TWOPI);

// chassis subsystem
src::chassis::ChassisSubsystem chassisSubsystem(
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
    &yawMotorBottom);

src::chassis::ChassisDriveCommand chassisDriveCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface);

src::chassis::ChassisOrientDriveCommand chassisOrientDriveCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface);

src::chassis::ChassisBeybladeCommand chassisBeyBladeCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    1,
    -1,
    2,
    true);

// chassis Mappings
ToggleCommandMapping beyBlade(
    drivers(),
    {&chassisBeyBladeCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::B})));

ToggleCommandMapping orientDrive(
    drivers(),
    {&chassisOrientDriveCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::R})));

// imu commands
imu::SentryImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {{
        &sentryTurrets,
        &chassisFrameYawTurretControllerTop,
        &chassisFramePitchTurretControllerTop,
        &chassisFrameYawTurretControllerBottom,
        &chassisFramePitchTurretControllerBottom,
        false,
        false,
    }},
    &chassisSubsystem);

// flywheel
// RevMotorTester revMotorTester(drivers());

FlywheelSubsystem flywheel(drivers(), LEFT_MOTOR_ID, RIGHT_MOTOR_ID, UP_MOTOR_ID, CAN_BUS);

FlywheelRunCommand flywheelRunCommand(&flywheel);

ToggleCommandMapping fPressed(
    drivers(),
    {&flywheelRunCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::F})));

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());
void initializeSubsystems(Drivers *drivers)
{
    chassisSubsystem.initialize();
    agitator.initialize();
    flywheel.initialize();
    sentryTurrets.initialize();
}

void registerSentrySubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassisSubsystem);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&flywheel);
    drivers->commandScheduler.registerSubsystem(&sentryTurrets);
}

void setDefaultSentryCommands(Drivers *drivers)
{
    chassisSubsystem.setDefaultCommand(&chassisDriveCommand);
    // m_FlyWheel.setDefaultCommand(&m_FlyWheelCommand);
    sentryTurrets.setDefaultCommand(&turretWRChassisImuCommand);
}

void startSentryCommands(Drivers *drivers)
{
    drivers->bmi088.setMountingTransform(
        tap::algorithms::transforms::Transform(0, 0, 0, 0, modm::toRadian(-45), 0));
    // drivers->commandScheduler.addCommand(&imuCalibrateCommand);
}

void registerSentryIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&leftMousePressed);
    drivers->commandMapper.addMap(&fPressed);
    // drivers.commandMapper.addMap(&rightMousePressed);
    // drivers.commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&beyBlade);
    drivers->commandMapper.addMap(&orientDrive);
}
}  // namespace sentry_control

namespace src::sentry
{
imu::ImuCalibrateCommandBase *getImuCalibrateCommand()
{
    return &sentry_control::imuCalibrateCommand;
}

void initSubsystemCommands(src::sentry::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &sentry_control::remoteSafeDisconnectFunction);
    sentry_control::initializeSubsystems(drivers);
    sentry_control::registerSentrySubsystems(drivers);
    sentry_control::setDefaultSentryCommands(drivers);
    sentry_control::startSentryCommands(drivers);
    sentry_control::registerSentryIoMappings(drivers);
}
}  // namespace src::sentry

#endif