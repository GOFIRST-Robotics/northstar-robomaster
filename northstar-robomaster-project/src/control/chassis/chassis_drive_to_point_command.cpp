#include "chassis_drive_to_point_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"


using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisDriveToPointCommand::ChassisDriveToPointCommand(
    ChassisSubsystem* chassis,
    src::chassis::ChassisOdometry* chassisOdometry,
    float xPosition,
    float yPosition,
    float maxError)
    : chassis(chassis),
      chassisOdometry(chassisOdometry),
      maxError(maxError)

{
    addSubsystemRequirement(chassis);
    targetPosition = modm::Vector<float, 2>(xPosition, yPosition);
}

void ChassisDriveToPointCommand::initialize() {}

void ChassisDriveToPointCommand::execute()
{
    auto scale = [](float raw) -> float { return limitVal(raw, -1.0f, 1.0f) * 0.5f; };
    modm::Vector<float, 2> dirToTarget = (targetPosition - chassisOdometry->getPositionGlobal());

    float length = dirToTarget.getLength();
    if (length > 1)
    {
        dirToTarget /= length;
    }
    else if (length < 0.35f)
    {
        dirToTarget = dirToTarget.normalized() * 0.35f;
    }

    chassis->setVelocityFieldDrive(dirToTarget.y, -dirToTarget.x, 0);
}

void ChassisDriveToPointCommand::end(bool interrupted) { chassis->setVelocityTurretDrive(0, 0, 0); }

bool ChassisDriveToPointCommand::isFinished() const
{
    return chassisOdometry->getPositionGlobal().getDistanceTo(targetPosition) <= maxError;
}

};  // namespace src::chassis