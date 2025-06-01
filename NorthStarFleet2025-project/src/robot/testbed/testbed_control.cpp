#ifdef TARGET_TEST_BED


#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "drivers_singleton.hpp"
#include "robot/testbed/testbed_drivers.hpp"




// safe disconnect
#include "control/safe_disconnect.hpp"

#include "communication/RevMotorTesterSingleMotor.hpp"

src::testbed::driversFunc drivers = src::testbed::DoNotUse_getDrivers;



namespace testbed_control
{

Communications::Rev::RevMotorTesterSingleMotor revMotorTesterSingleMotor(drivers());

src::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

void initializeSubsystems(src::testbed::Drivers *drivers)
{
    revMotorTesterSingleMotor.initialize();
}

void registerStandardSubsystems(src::testbed::Drivers *drivers)
{
}

void setDefaultStandardCommands(src::testbed::Drivers *drivers)
{
}

void startStandardCommands(src::testbed::Drivers *drivers)
{
}

void registerStandardIoMappings(src::testbed::Drivers *drivers)
{
}
}  // namespace standard_control

namespace src::testbed
{
void initSubsystemCommands(src::testbed::Drivers *drivers)
{
    testbed_control::initializeSubsystems(drivers);
}
}  // namespace src::standard

#endif