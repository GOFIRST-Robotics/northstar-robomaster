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
        return;
    }

    modm::Vector<float, 2> dirToTarget = (path[0] - chassisOdometry->getPositionGlobal());

    float distanceToTarget = dirToTarget.getLength();
    modm::Vector<float, 2> velocityToTarget;

    if (distanceToTarget > MAXIMUM_MPS)
    {
        velocityToTarget = dirToTarget.normalized() * MAXIMUM_MPS;
    }
    else if (distanceToTarget < MINIMUM_MPS)
    {
        velocityToTarget = dirToTarget.normalized() * MINIMUM_MPS;
    }
    else
    {
        velocityToTarget = dirToTarget;
    }

    chassis->setVelocityFieldDrive(velocityToTarget.y, -velocityToTarget.x, 0);
}

};  // namespace src::chassis