#include "chassis_auto_drive.hpp"

namespace src::chassis
{
ChassisAutoDrive::ChassisAutoDrive(
    ChassisSubsystem* chassis,
    src::chassis::ChassisOdometry* chassisOdometry)
    : chassis(chassis),
      chassisOdometry(chassisOdometry),
      path(std::deque<modm::Vector<float, 2>>())

{
}

void ChassisAutoDrive::resetPath() { path.clear(); }

void ChassisAutoDrive::addPointToPath(modm::Vector<float, 2> newPoint) { path.push_back(newPoint); }

void ChassisAutoDrive::updateAutoDrive()
{
    if (!tryUpdatePath())
    {
        desiredGlobalVelocity = modm::Vector<float, 2>(0, 0);
        return;
    }

    modm::Vector<float, 2> dirToTarget = (path[0] - chassisOdometry->getPositionGlobal());

    float distanceToTarget = dirToTarget.getLength();
    modm::Vector<float, 2> velocityToTarget = dirToTarget;

    if (distanceToTarget > MAXIMUM_MPS)
    {
        velocityToTarget = (dirToTarget / distanceToTarget) * MAXIMUM_MPS;
    }
    else if (distanceToTarget < MINIMUM_MPS)
    {
        velocityToTarget = (dirToTarget / distanceToTarget) * MINIMUM_MPS;
    }

    velocityToTarget.x = -velocityToTarget.x;  // flipped for some reason??
    currentIdealVelocity = velocityToTarget;

    desiredGlobalVelocity =
        desiredGlobalVelocity + (currentIdealVelocity - desiredGlobalVelocity).normalized() * 0.003;

    desiredRotation = desiredRotation + chassisOdometry->getRotation() * 0.1;
    if (desiredRotation > .8)
    {
        desiredRotation = .8;
    }
    else if (desiredRotation < -.8)
    {
        desiredRotation = -.8;
    }
}

};  // namespace src::chassis