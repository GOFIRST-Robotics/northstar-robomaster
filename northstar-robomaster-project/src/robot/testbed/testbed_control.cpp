#ifdef TARGET_TEST_BED

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "control/agitator/multi_shot_cv_command_mapping.hpp"
#include "control/cycle_state_command_mapping.hpp"
#include "control/dummy_subsystem.hpp"
#include "robot/testbed/testbed_drivers.hpp"

#include "drivers_singleton.hpp"

// imu
#include "control/imu/imu_calibrate_command.hpp"

// agitator
#include "control/agitator/constant_velocity_agitator_command.hpp"
#include "control/agitator/constants/agitator_constants.hpp"
#include "control/agitator/manual_fire_rate_reselection_manager.hpp"
#include "control/agitator/set_fire_rate_command.hpp"
#include "control/agitator/unjam_spoke_agitator_command.hpp"
#include "control/agitator/velocity_agitator_subsystem.hpp"

// flywheel
#include "control/flywheel/flywheel_constants.hpp"
#include "control/flywheel/flywheel_run_command.hpp"
#include "control/flywheel/flywheel_subsystem.hpp"

// chassis
#include "control/chassis/chassis_beyblade_command.hpp"
#include "control/chassis/chassis_drive_command.hpp"
#include "control/chassis/chassis_orient_drive_command.hpp"
#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/chassis_wiggle_command.hpp"
#include "control/chassis/constants/chassis_constants.hpp"

// safe disconnect
#include "communication/RevMotorTesterSingleMotor.hpp"
#include "control/safe_disconnect.hpp"

// turret
#include "control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "control/turret/constants/turret_constants.hpp"
#include "control/turret/user/turret_user_world_relative_command.hpp"

// standard turret
#include "control/turret/CV/turret_cv_control_command.hpp"
#include "control/turret/algorithms/world_frame_turret_can_imu_turret_controller.hpp"
#include "control/turret/user/turret_quick_turn_command.hpp"
#include "control/turret/user/turret_user_control_command.hpp"
#include "robot/standard/standard_turret_subsystem.hpp"

// sentry turret
#include "robot/sentry/sentry_turret_subsystem.hpp"
#include "robot/sentry/sentry_turret_user_world_relative_command.hpp"

// NEO turret
#include "control/turret/rev_turret_subsystem.hpp"
#include "control/turret/user/neo_turret_user_control_command.hpp"

// governor
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"

#include "control/governor/fire_rate_limit_governor.hpp"
#include "control/governor/fired_recently_governor.hpp"
#include "control/governor/flywheel_on_governor.hpp"
#include "control/governor/heat_limit_governor.hpp"
#include "control/governor/plate_hit_governor.hpp"
#include "control/governor/ref_system_projectile_launched_governor.hpp"

#include "ref_system_constants.hpp"

src::testbed::driversFunc drivers = src::testbed::DoNotUse_getDrivers;

using namespace tap::control::setpoint;
using namespace tap::control;
using namespace src::testbed;
// using namespace src::control::turret;
using namespace src::control;
using namespace src::flywheel;
using namespace src::control::flywheel;
using namespace src::agitator;
using namespace src::control::agitator;
using namespace src::control::turret;
using namespace src::control::governor;
using namespace tap::control::governor;
using namespace tap::communication::serial;

// what to test
// #define FLYWHEEL_TEST
// #define REV_TEST
// #define AGITATOR_TEST
// #define SENTRY_TURRET_TEST
// #define SENTRY_CONSTANTS
#define NEO_TURRET_TEST

// #define CHASSIS_TEST
// #define HERO_CHASSIS_CONSTANTS
// #define STANDARD_TURRET_TEST

namespace testbed_control
{
DummySubsystem dummySubsystem(drivers());

inline src::can::TurretMCBCanComm &getTurretMCBCanComm() { return drivers()->turretMCBCanCommBus2; }
src::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

#ifdef REV_TEST
Communications::Rev::RevMotorTesterSingleMotor revMotorTesterSingleMotor(drivers());

#endif

#ifdef FLYWHEEL_TEST
FlywheelSubsystem flywheel(drivers(), LEFT_MOTOR_ID, RIGHT_MOTOR_ID, UP_MOTOR_ID, CAN_BUS);

// flywheel commands
FlywheelRunCommand flywheelRunCommand(&flywheel);

// flywheel mappings
ToggleCommandMapping fPressed(
    drivers(),
    {&flywheelRunCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::F})));
#endif  // FLYWHEEL_TEST

#ifdef AGITATOR_TEST
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

// agitator governors
ManualFireRateReselectionManager manualFireRateReselectionManager;

SetFireRateCommand setFireRateCommandFullAuto(
    &dummySubsystem,
    manualFireRateReselectionManager,
    40,
    &rotateAgitator);
SetFireRateCommand setFireRateCommand10RPS(
    &dummySubsystem,
    manualFireRateReselectionManager,
    10,
    &rotateAgitator);
SetFireRateCommand setFireRateCommand30RPS(
    &dummySubsystem,
    manualFireRateReselectionManager,
    30,
    &rotateAgitator);

ToggleCommandMapping qPressed10RPS(
    drivers(),
    {&setFireRateCommand10RPS},
    RemoteMapState(RemoteMapState({Remote::Key::Q})));

ToggleCommandMapping wPressed30RPS(
    drivers(),
    {&setFireRateCommand30RPS},
    RemoteMapState(RemoteMapState({Remote::Key::Q})));

ToggleCommandMapping ePressedFullAuto(
    drivers(),
    {&setFireRateCommandFullAuto},
    RemoteMapState(RemoteMapState({Remote::Key::W})));

FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

GovernorLimitedCommand<1> rotateAndUnjamAgitatorLimited(
    {&agitator},
    rotateAndUnjamAgitator,
    {&fireRateLimitGovernor});

HoldRepeatCommandMapping leftMousePressedShoot(
    drivers(),
    {&rotateAndUnjamAgitatorLimited},
    RemoteMapState(RemoteMapState::MouseButton::LEFT),
    false);

HoldRepeatCommandMapping leftSwitchDownPressedShoot(
    drivers(),
    {&rotateAndUnjamAgitatorLimited},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    false);

#endif  // AGITATOR_TEST

#ifdef NEO_TURRET_TEST

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
    tap::motor::REV_MOTOR1,
    CanBus::CAN_BUS1,
    false,
    "YawMotor1",
    18.0f / 120.0f);  // gear ratio

tap::motor::RevMotor yawMotor2(
    drivers(),
    tap::motor::REV_MOTOR2,
    CanBus::CAN_BUS1,
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

#ifdef STANDARD_TURRET_TEST
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

StandardTurretSubsystem turret(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

// turret controlers
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
    turret.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    turret.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
    *drivers(),
    turret.yawMotor,
    world_rel_chassis_imu::YAW_PID_CONFIG);

algorithms::WorldFramePitchChassisImuTurretController worldFramePitchChassisImuController(
    *drivers(),
    turret.pitchMotor,
    world_rel_chassis_imu::PITCH_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretPosPid(world_rel_turret_imu::PITCH_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretVelPid(world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

// for imu can com giving imu data from turret to chassis
algorithms::
    WorldFramePitchTurretCanImuCascadePidTurretController worldFramePitchTurretCanImuController(
        getTurretMCBCanComm(),
        turret.pitchMotor,
        worldFramePitchTurretPosPid,
        worldFramePitchTurretVelPid);

algorithms::WorldFrameYawTurretCanImuCascadePidTurretController worldFrameYawTurretCanImuController(
    getTurretMCBCanComm(),
    turret.yawMotor,
    worldFrameYawTurretPosPid,
    worldFrameYawTurretVelPid);

// for imu fixed on turret
algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
    *drivers(),
    turret.pitchMotor,
    worldFramePitchTurretPosPid,
    worldFramePitchTurretVelPid);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
    *drivers(),
    turret.yawMotor,
    worldFrameYawTurretPosPid,
    worldFrameYawTurretVelPid);

// turret commands
user::TurretUserControlCommand turretUserControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turret,
    &worldFrameYawTurretImuController,
    &worldFramePitchChassisImuController,  //&worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

cv::TurretCVControlCommand turretCVControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    drivers()->visionComms,
    &turret,
    &worldFrameYawTurretImuController,
    &worldFramePitchChassisImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

user::TurretUserWorldRelativeCommand turretUserWorldRelativeCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turret,
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

#ifdef SENTRY_TURRET_TEST
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
    false,
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
    false,
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

// // turret controlers
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
    world_rel_turret_imu::PITCH_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretImuVelPidBottom(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

algorithms::
    WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuControllerBottom(
        *drivers(),
        sentryTurrets.pitchMotorBottom,
        worldFramePitchTurretImuPosPidBottom,
        worldFramePitchTurretImuVelPidBottom);

tap::algorithms::SmoothPid worldFramePitchTurretImuPosPidTop(
    world_rel_turret_imu::PITCH_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretImuVelPidTop(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

algorithms::
    WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuControllerTop(
        *drivers(),
        sentryTurrets.pitchMotorTop,
        worldFramePitchTurretImuPosPidTop,
        worldFramePitchTurretImuVelPidTop);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPidBottom(
    world_rel_turret_imu::YAW_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretImuVelPidBottom(
    world_rel_turret_imu::YAW_VEL_PID_CONFIG);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuControllerBottom(
    *drivers(),
    sentryTurrets.yawMotorBottom,
    worldFrameYawTurretImuPosPidBottom,
    worldFrameYawTurretImuVelPidBottom);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPidTop(
    world_rel_turret_imu::YAW_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretImuVelPidTop(
    world_rel_turret_imu::YAW_VEL_PID_CONFIG);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuControllerTop(
    *drivers(),
    sentryTurrets.yawMotorTop,
    worldFrameYawTurretImuPosPidTop,
    worldFrameYawTurretImuVelPidTop);

// // turret commands
user::SentryTurretUserControlCommand turretWRChassisImuCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &sentryTurrets,
    &worldFrameYawChassisImuControllerBottom,
    &worldFramePitchChassisImuControllerBottom,
    &chassisFrameYawTurretControllerTop,  // controler for top turret
    &worldFramePitchChassisImuControllerTop,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

#endif

#ifdef CHASSIS_TEST
FiredRecentlyGovernor firedRecentlyGovernor(drivers(), 5000);

PlateHitGovernor plateHitGovernor(drivers(), 5000);

// GovernorWithFallbackCommand<2> beyBladeSlowOutOfCombat(
//     {&chassisSubsystem},
//     chassisBeyBladeSlowCommand,
//     chassisBeyBladeFastCommand,
//     {&firedRecentlyGovernor, &plateHitGovernor},
//     true);

// chassis Mappings
ToggleCommandMapping bPressed(
    drivers(),
    {&beyBladeSlowOutOfCombat},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::B})));

// imu::ImuCalibrateCommand imuCalibrateCommand(
//     drivers(),
//     {{
//         &turret,
//         &chassisFrameYawTurretController,
//         &chassisFramePitchTurretController,
//         true,
//     }},
//     &chassisSubsystem);

ToggleCommandMapping ctrlCPressed(
    drivers(),
    {&imuCalibrateCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::C})));

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

src::chassis::ChassisBeybladeCommand chassisBeyBladeSlowCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    1,
    -1,
    1,
    true);

src::chassis::ChassisBeybladeCommand chassisBeyBladeFastCommand(
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

void initializeSubsystems(src::testbed::Drivers *drivers)
{
    dummySubsystem.initialize();
#ifdef AGITATOR_TEST
    agitator.initialize();
#endif
#ifdef FLYWHEEL_TEST
    flywheel.initialize();
#endif
#ifdef SENTRY_TURRET_TEST
    sentryTurrets.initialize();
#endif
#ifdef STANDARD_TURRET_TEST
    turret.initialize();
#endif  // STANDARD_TURRET_TEST
#ifdef CHASSIS_TEST
    chassisSubsystem.initialize();
#endif
#ifdef REV_TEST
    revMotorTesterSingleMotor.initialize();
#endif
#ifdef NEO_TURRET_TEST
    revTurret.initialize();
#endif
}

void registerTestSubsystems(src::testbed::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&dummySubsystem);

#ifdef AGITATOR_TEST
    drivers->commandScheduler.registerSubsystem(&agitator);
#endif
#ifdef FLYWHEEL_TEST
    drivers->commandScheduler.registerSubsystem(&flywheel);
#endif
#ifdef SENTRY_TURRET_TEST
    drivers->commandScheduler.registerSubsystem(&sentryTurrets);
#endif
#ifdef STANDARD_TURRET_TEST
    drivers->commandScheduler.registerSubsystem(&turret);
#endif  // STANDARD_TURRET_TEST
#ifdef CHASSIS_TEST
    drivers->commandScheduler.registerSubsystem(&chassisSubsystem);
#endif
#ifdef REV_TEST
    drivers->commandScheduler.registerSubsystem(&revMotorTesterSingleMotor);
#endif
#ifdef NEO_TURRET_TEST
    drivers->commandScheduler.registerSubsystem(&revTurret);
#endif
}

void setDefaultTestCommands(src::testbed::Drivers *drivers)
{
#ifdef SENTRY_TURRET_TEST
    sentryTurrets.setDefaultCommand(&turretWRChassisImuCommand);
#endif  // SENTRY_TURRET_TEST
#ifdef STANDARD_TURRET_TEST
    turret.setDefaultCommand(&turretUserControlCommand);
#endif  // STANDARD_TURRET_TEST
#ifdef CHASSIS_TEST
    chassisSubsystem.setDefaultCommand(&chassisOrientDriveCommand);
#endif
#ifdef NEO_TURRET_TEST
    revTurret.setDefaultCommand(&turretUserControlCommand);
#endif
}

void startTestCommands(src::testbed::Drivers *drivers)
{
    // drivers->bmi088.setMountingTransform(
    //     tap::algorithms::transforms::Transform(0, 0, 0, 0, modm::toRadian(-45), 0));
}

void registerTestIoMappings(src::testbed::Drivers *drivers)
{
#ifdef AGITATOR_TEST
    drivers->commandMapper.addMap(&leftMousePressedShoot);
    drivers->commandMapper.addMap(&leftSwitchDownPressedShoot);
    drivers->commandMapper.addMap(&qPressed10RPS);
    drivers->commandMapper.addMap(&wPressed30RPS);
    drivers->commandMapper.addMap(&ePressedFullAuto);

#endif
#ifdef FLYWHEEL_TEST
    drivers->commandMapper.addMap(&fPressed);
#endif
#ifdef STANDARD_TURRET_TEST
    drivers->commandMapper.addMap(&xPressed);
#endif  // STANDARD_TURRET_TEST
    // drivers->commandMapper.addMap(&ctrlCPressed);
#ifdef CHASSIS_TEST
    drivers->commandMapper.addMap(&bPressed);
#endif
}
}  // namespace testbed_control

namespace src::testbed
{
// imu::ImuCalibrateCommandBase *getImuCalibrateCommand() { return
// &testbed_control::imuCalibrateCommand; }

void initSubsystemCommands(src::testbed::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &testbed_control::remoteSafeDisconnectFunction);
    testbed_control::initializeSubsystems(drivers);
    testbed_control::registerTestSubsystems(drivers);
    testbed_control::setDefaultTestCommands(drivers);
    testbed_control::startTestCommands(drivers);
    testbed_control::registerTestIoMappings(drivers);
}
}  // namespace src::testbed

#endif