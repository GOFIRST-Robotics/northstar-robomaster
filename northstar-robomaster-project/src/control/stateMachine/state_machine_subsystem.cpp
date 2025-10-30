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

    modm::Vector<float, 2> directionToCurve = chassisAutoDrive->getDirectionToCurve(t);
    float lengthToCurve = directionToCurve.getLength();

    if (lengthToCurve < 0.05)
    {
        t += 0.02f;
        return;
    }

    modm::Vector<float, 2> deriv = chassisAutoDrive->getLookaheadDeriv(t, 0.05);
    float desiredFacingRadians = atan2(deriv.y, deriv.x);
    float desiredDriveRotation =
        (desiredFacingRadians - chassisAutoDrive->getOdometryRotation() - (M_PI_2)) * -1.2;

    modm::Vector<float, 2> desiredVelocity = directionToCurve.normalized() * 0.6f;
    chassisSubsystem->setVelocityFieldDrive(
        desiredVelocity.y,
        -desiredVelocity.x,
        desiredDriveRotation);

    /*if (chassisAutoDrive->getPath().size() == 0)
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
        chassisAutoDrive->getOdometryRotation());*/
}

}  // namespace src::stateMachine