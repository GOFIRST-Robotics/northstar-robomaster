#include "chassis_field_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisFieldCommand::ChassisFieldCommand(
    ChassisSubsystem* chassis,
    src::control::ControlOperatorInterface* operatorInterface)
    : chassis(chassis),
      operatorInterface(operatorInterface)
{
    addSubsystemRequirement(chassis);
}

void ChassisFieldCommand::execute()
{
    auto scale = [](float raw) -> float {
        return limitVal(raw, -1.0f, 1.0f) * MAX_CHASSIS_SPEED_MPS;
    };
    modm::Pair<float, float> normInput = getNormalizedInput(
        operatorInterface->getDrivetrainVerticalTranslation(),
        operatorInterface->getDrivetrainHorizontalTranslation());
    chassis->setVelocityFieldDrive(
        scale(normInput.first),
        -scale(normInput.second),
        scale(operatorInterface->getDrivetrainRotationalTranslation()));
}

void ChassisFieldCommand::end(bool interrupted) { chassis->setVelocityTurretDrive(0, 0, 0); }
};  // namespace src::chassis