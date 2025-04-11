#ifdef TARGET_HERO

#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "robot/hero/hero_drivers.hpp"
#include "robot/hero/hero_flywheel_constants.hpp"
#include "robot/hero/hero_flywheel_subsystem.hpp"

#include "drivers_singleton.hpp"

using tap::can::CanBus;

using namespace src::hero;
using namespace src::control;
using namespace src::control::flywheel;

driversFunc drivers = DoNotUse_getDrivers;

namespace hero_control
{
HeroFlywheelSubsystem flywheel(drivers(), LEFT_MOTOR_ID, RIGHT_MOTOR_ID, UP_MOTOR_ID, CAN_BUS);

void initializeSubsystems(Drivers *drivers) { flywheel.initialize(); }

void registerSoldierSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&flywheel);
}

void setDefaultSoldierCommands(Drivers *drivers) {}

void startSoldierCommands(Drivers *drivers) {}

void registerSoldierIoMappings(Drivers *drivers) {}
}  // namespace hero_control

namespace src::hero
{
void initSubsystemCommands(src::hero::Drivers *drivers)
{
    hero_control::initializeSubsystems(drivers);
    hero_control::registerSoldierSubsystems(drivers);
    hero_control::setDefaultSoldierCommands(drivers);
    hero_control::startSoldierCommands(drivers);
    hero_control::registerSoldierIoMappings(drivers);
}
}  // namespace src::hero

#endif  // TARGET_HERO