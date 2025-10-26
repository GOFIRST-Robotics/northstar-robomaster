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
void StateMachineSubsystem::refresh()
{
    if (drivers->remote.getChannel(5))
    {
        chassisSubsystem->setVelocityFieldDrive(0, 0, 0);
        return;
    }

    if (chassisAutoDrive->getPath().size() == 0)
    {
        chassisAutoDrive->addPointToPath(
            a ? modm::Vector<float, 2>(0, 0) : modm::Vector<float, 2>(0, 1.0f));
        a = !a;
    }

    chassisAutoDrive->updateAutoDrive();

    modm::Vector<float, 2> autoDriveDesiredGlobalVelocity =
        chassisAutoDrive->getDesiredGlobalVelocity();
    float autoDriveDesiredRotation = chassisAutoDrive->getDesiredRotation();
    // chassisSubsystem->setVelocityFieldDrive(
    //     autoDriveDesiredGlobalVelocity.y,
    //     autoDriveDesiredGlobalVelocity.x,
    //     0);

    chassisSubsystem->driveBasedOnHeading(
        autoDriveDesiredGlobalVelocity.y,
        autoDriveDesiredGlobalVelocity.x,
        autoDriveDesiredRotation,
        chassisAutoDrive->getOdometryRotation());
}

}  // namespace src::stateMachine