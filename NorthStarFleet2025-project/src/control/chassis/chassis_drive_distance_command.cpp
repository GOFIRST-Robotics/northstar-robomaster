#include "chassis_drive_distance_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisDriveDistanceCommand::ChassisDriveDistanceCommand(
    ChassisSubsystem* chassis,
    src::control::ControlOperatorInterface* operatorInterface,
    float xDist,
    float yDist,
    float maxError)
    : chassis(chassis),
      operatorInterface(operatorInterface),
      xDist(xDist),
      yDist(yDist),
      maxError(maxError)

{
    addSubsystemRequirement(chassis);
    xPid = Pid(2.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    yPid = Pid(2.0f, 0.0f, 0.0f, 0.0f, 1.f);
}

void ChassisDriveDistanceCommand::initialize()
{
    prevTime = tap::arch::clock::getTimeMilliseconds();
    xDistanceCounter = 0.0f;
    yDistanceCounter = 0.0f;
}

void ChassisDriveDistanceCommand::execute()
{
    auto scale = [](float raw) -> float { return limitVal(raw, -1.0f, 1.0f) * 0.5f; };
    uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = curTime - prevTime;
    prevTime = curTime;
    xPid.update(xDist - xDistanceCounter);
    yPid.update(yDist - yDistanceCounter);
    chassis->setVelocityTurretDrive(scale(xPid.getValue()), scale(yPid.getValue()), 0);
    xDistanceCounter += xPid.getValue() * 0.5f * (dt / 1000.0);
    yDistanceCounter += yPid.getValue() * 0.5f * (dt / 1000.0);
}

void ChassisDriveDistanceCommand::end(bool interrupted)
{
    chassis->setVelocityTurretDrive(0, 0, 0);
}

bool ChassisDriveDistanceCommand::isFinished() const
{
    return abs(xDist - xDistanceCounter) <= maxError && abs(yDist - yDistanceCounter) <= maxError;
}

};  // namespace src::chassis