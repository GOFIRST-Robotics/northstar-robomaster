#include "state_machine_subsytem.hpp"

namespace src::stateMachine
{
StateMachineSubsystem::StateMachineSubsystem(
    tap::Drivers* drivers,
    src::chassis::ChassisSubsystem* chassisSubsystem,
    src::chassis::ChassisAutoDrive* chassisAutoDrive)
    : Subsystem(drivers),
      drivers(drivers),
      chassisSubsystem(chassisSubsystem),
      chassisAutoDrive(chassisAutoDrive)
{
}

void StateMachineSubsystem::initialize() {}

bool a = false;
float t = 0;

void StateMachineSubsystem::refresh()
{
    if (drivers->remote.getChannel(5) || t > 1)
    {
        chassisSubsystem->setVelocityFieldDrive(0, 0, 0);
        return;
    }

    if (!a)
    {
        a = true;
        chassisAutoDrive->addCurveToPath(CubicBezier(
            modm::Vector<float, 2>(0, 0),
            modm::Vector<float, 2>(0, 3.4),
            modm::Vector<float, 2>(0, 1),
            modm::Vector<float, 2>(0, 2.4)));
    }

    chassisAutoDrive->updateAutoDrive();
    modm::Vector<float, 2> desiredGlobalVelocity = chassisAutoDrive->getDesiredGlobalVelocity();
    float desiredRadiansPerSecond = chassisAutoDrive->getDesiredRotation();

    chassisSubsystem->setVelocityFieldDrive(
        desiredGlobalVelocity.y,
        desiredGlobalVelocity.x,
        desiredRadiansPerSecond);
}

}  // namespace src::stateMachine