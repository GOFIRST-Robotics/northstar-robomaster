#include "chassis_drive_distance_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisDriveDistanceCommand::ChassisDriveDistanceCommand(
    ChassisSubsystem* chassis,
    src::chassis::ChassisOdometry* chassisOdometry,
    float xDist,
    float yDist,
    float maxError)
    : chassis(chassis),
      chassisOdometry(chassisOdometry),
      maxError(maxError)

{
    addSubsystemRequirement(chassis);
    xPid = Pid(2.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    yPid = Pid(2.0f, 0.0f, 0.0f, 0.0f, 1.f);

    targetPosition = chassisOdometry->convertLocalToGlobal(modm::Vector<float, 2>(xDist, yDist));
}

void ChassisDriveDistanceCommand::initialize() {}

void ChassisDriveDistanceCommand::execute()
{
    auto scale = [](float raw) -> float { return limitVal(raw, -1.0f, 1.0f) * 0.5f; };
    xPid.update(targetPosition.x - chassisOdometry->getPositionGlobal().x);
    yPid.update(targetPosition.y - chassisOdometry->getPositionGlobal().y);
    chassis->setVelocityTurretDrive(scale(xPid.getValue()), scale(yPid.getValue()), 0);
}

void ChassisDriveDistanceCommand::end(bool interrupted)
{
    chassis->setVelocityTurretDrive(0, 0, 0);
}

bool ChassisDriveDistanceCommand::isFinished() const
{
    return chassisOdometry->getPositionGlobal().getDistanceTo(targetPosition) <= maxError;
}

};  // namespace src::chassis