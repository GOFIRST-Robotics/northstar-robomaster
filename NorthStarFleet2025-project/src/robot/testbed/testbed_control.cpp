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

// flywheel
#include "control/flywheel/flywheel_constants.hpp"
#include "control/flywheel/flywheel_run_command.hpp"
#include "control/flywheel/flywheel_subsystem.hpp"

// safe disconnect
#include "communication/RevMotorTesterSingleMotor.hpp"
#include "control/safe_disconnect.hpp"


src::testbed::driversFunc drivers = src::testbed::DoNotUse_getDrivers;

// using namespace tap::control::setpoint;
using namespace tap::control;
// using namespace src::standard;
// using namespace src::control::turret;
using namespace src::control;
using namespace src::flywheel;
using namespace src::control::flywheel;
// using namespace src::agitator;
// using namespace src::control::agitator;
// using namespace src::control::governor;
// using namespace tap::control::governor;

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

void initializeSubsystems(src::testbed::Drivers *drivers)
{
    revMotorTesterSingleMotor.initialize();
}

void registerTestSubsystems(src::testbed::Drivers *drivers) {}

void setDefaultTestCommands(src::testbed::Drivers *drivers) {}

void startTestCommands(src::testbed::Drivers *drivers) {}

void registerTestIoMappings(src::testbed::Drivers *drivers) {}
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