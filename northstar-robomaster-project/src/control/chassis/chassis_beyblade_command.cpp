#include "chassis_beyblade_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisBeybladeCommand::ChassisBeybladeCommand(
    ChassisSubsystem* chassis,
    src::control::ControlOperatorInterface* operatorInterface,
    float distScaleFactor,
    short direction,
    float spinVel,
    bool isVariable)
    : chassis(chassis),
      operatorInterface(operatorInterface),
      distScaleFactor(distScaleFactor),
      direction(direction),
      spinVel(spinVel),
      isVariable(isVariable)
{
    addSubsystemRequirement(chassis);
}

void ChassisBeybladeCommand::initialize()
{
    prevTime = tap::arch::clock::getTimeMilliseconds();
    calcSpeed = 1.0f * direction;
}

float calcedRot;

void ChassisBeybladeCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    auto scale = [](float raw) -> float {
        return limitVal(raw, -1.0f, 1.0f) * MAX_CHASSIS_SPEED_MPS;
    };
    float verticalSpeed = scale(operatorInterface->getDrivetrainVerticalTranslation());
    float horizontalSpeed = -scale(operatorInterface->getDrivetrainHorizontalTranslation());
    calcedRot = calculateBeyBladeRotationSpeed(
        chassis->calculateMaxRotationSpeed(verticalSpeed, horizontalSpeed),
        dt);
    chassis->setVelocityTurretDrive(verticalSpeed, horizontalSpeed, calcedRot);
}

void ChassisBeybladeCommand::end(bool interrupted) { chassis->setVelocityTurretDrive(0, 0, 0); }

float ChassisBeybladeCommand::calculateBeyBladeRotationSpeed(float maxSpeed, uint32_t dt)
{
    accumTime += dt;
    if (!isVariable)
    {
        return maxSpeed * direction;
        // return limitVal<float>(1.0f, 0.0f, 1.0f) * direction;
    }
    RandomNumberGenerator::enable();

    if (accumTime > 500)
    {
        calcSpeed = limitVal<float>(1.0f, 0.0f, 0.9f) * direction;
        if (RandomNumberGenerator::isReady())
        {
            calcSpeed = limitVal<float>(
                0.1 * sin(RandomNumberGenerator::getValue()) + calcSpeed,
                -1.0f,
                1.0f);
        }
        accumTime = 0;
    }
    return calcSpeed * maxSpeed * direction;
}
};  // namespace src::chassis