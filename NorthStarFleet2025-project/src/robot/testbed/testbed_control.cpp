#ifdef TARGET_TEST_BED

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "control/dummy_subsystem.hpp"
#include "robot/testbed/testbed_drivers.hpp"

#include "drivers_singleton.hpp"

// agitator
#include "control/agitator/constant_velocity_agitator_command.hpp"
#include "control/agitator/constants/agitator_constants.hpp"
#include "control/agitator/set_fire_rate_command.hpp"
#include "control/agitator/unjam_spoke_agitator_command.hpp"
#include "control/agitator/velocity_agitator_subsystem.hpp"

// flywheel
#include "control/flywheel/flywheel_constants.hpp"
#include "control/flywheel/flywheel_run_command.hpp"
#include "control/flywheel/flywheel_subsystem.hpp"

// safe disconnect
#include "communication/RevMotorTesterSingleMotor.hpp"
#include "control/safe_disconnect.hpp"

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
// using namespace src::control::governor;
// using namespace tap::control::governor;

// what to test
#define FLYWHEEL_TEST
#define AGITATOR_TEST

namespace testbed_control
{
Communications::Rev::RevMotorTesterSingleMotor revMotorTesterSingleMotor(drivers());

src::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

FlywheelSubsystem flywheel(drivers(), LEFT_MOTOR_ID, RIGHT_MOTOR_ID, UP_MOTOR_ID, CAN_BUS);

// flywheel commands
FlywheelRunCommand flywheelRunCommand(&flywheel);

// flywheel mappings
ToggleCommandMapping fPressed(
    drivers(),
    {&flywheelRunCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::F})));

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
// HeatLimitGovernor heatLimitGovernor(
//     *drivers(),
//     tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1,
//     constants::HEAT_LIMIT_BUFFER);

// FlywheelOnGovernor flywheelOnGovernor(flywheel);

// RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernor(
//     drivers()->refSerial,
//     tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

// ManualFireRateReselectionManager manualFireRateReselectionManager;

// SetFireRateCommand setFireRateCommandFullAuto(
//     &dummySubsystem,
//     manualFireRateReselectionManager,
//     40,
//     &rotateAgitator);
// SetFireRateCommand setFireRateCommand10RPS(
//     &dummySubsystem,
//     manualFireRateReselectionManager,
//     10,
//     &rotateAgitator);

// FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

// GovernorLimitedCommand<3> rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched(
//     {&agitator},
//     rotateAndUnjamAgitator,
//     {&refSystemProjectileLaunchedGovernor, &fireRateLimitGovernor, &flywheelOnGovernor});

// agitator mappings
// ToggleCommandMapping vPressed(
//     drivers(),
//     {&setFireRateCommandFullAuto},
//     RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::V})));

// ToggleCommandMapping gPressed(
//     drivers(),
//     {&setFireRateCommand10RPS},
//     RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::G})));

HoldRepeatCommandMapping leftMousePressed(
    drivers(),
    {&rotateAndUnjamAgitator},  // TODO
    RemoteMapState(RemoteMapState::MouseButton::LEFT),
    false);

void initializeSubsystems(src::testbed::Drivers *drivers)
{
#ifdef AGITATOR_TEST
    agitator.initialize();
#endif
#ifdef FLYWHEEL_TEST
    flywheel.initialize();
#endif
    // revMotorTesterSingleMotor.initialize();
}

void registerTestSubsystems(src::testbed::Drivers *drivers)
{
#ifdef AGITATOR_TEST
    drivers->commandScheduler.registerSubsystem(&agitator);
#endif
#ifdef FLYWHEEL_TEST
    drivers->commandScheduler.registerSubsystem(&flywheel);
#endif
}

void setDefaultTestCommands(src::testbed::Drivers *drivers) {}

void startTestCommands(src::testbed::Drivers *drivers) {}

void registerTestIoMappings(src::testbed::Drivers *drivers)
{
#ifdef AGITATOR_TEST
    drivers->commandMapper.addMap(&leftMousePressed);
#endif
#ifdef FLYWHEEL_TEST
    drivers->commandMapper.addMap(&fPressed);
#endif
}
}  // namespace testbed_control

namespace src::testbed
{
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