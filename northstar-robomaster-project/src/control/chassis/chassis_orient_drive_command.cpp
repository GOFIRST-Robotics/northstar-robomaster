#include "chassis_orient_drive_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisOrientDriveCommand::ChassisOrientDriveCommand(
    ChassisSubsystem* chassis,
    src::control::ControlOperatorInterface* operatorInterface)
    : chassis(chassis),
      operatorInterface(operatorInterface)
{
    addSubsystemRequirement(chassis);
    rotationalValue = chassis->getChassisRotationSpeed();
}

void ChassisOrientDriveCommand::execute()
{
    auto scale = [](float raw) -> float {
        return limitVal(raw, -1.0f, 1.0f) * MAX_CHASSIS_SPEED_MPS;
    };

    float rotationFromPID = chassis->chassisSpeedRotationPID();

    float rotationalAlpha =
        std::max<float>(1.0f - abs(chassis->getChassisZeroTurret()) / M_PI, 0.2f);

    rotationalValue =
        tap::algorithms::lowPassFilter(rotationalValue, rotationFromPID, rotationalAlpha);

    chassis->setVelocityTurretDrive(
        scale(operatorInterface->getDrivetrainVerticalTranslation()),
        -scale(operatorInterface->getDrivetrainHorizontalTranslation()),
        scale(rotationalValue));
}

void ChassisOrientDriveCommand::end(bool interrupted) { chassis->setVelocityTurretDrive(0, 0, 0); }
};  // namespace src::chassis