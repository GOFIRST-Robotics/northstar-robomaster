#include "chassis_drive_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

using tap::algorithms::limitVal;

namespace control::chassis
{
// STEP 1 (Tank Drive): Constructor
    ChassisDriveCommand::ChassisDriveCommand(ChassisSubsystem* chassis, src::control::ControlOperatorInterface* operatorInterface) :
    chassis(chassis),
    operatorInterface(operatorInterface)
    {
        addSubsystemRequirement(chassis);
    }
// STEP 2 (Tank Drive): execute function
    void ChassisDriveCommand::execute() {
        auto scale = [](float raw) -> float {
            return limitVal(raw, -1.0f, 1.0f) * MAX_CHASSIS_SPEED_MPS;
        };
        #ifdef FIELD
        chassis->setVelocityDrive(
            scale(operatorInterface->getDrivetrainVerticalTranslation()),
            scale(operatorInterface->getDrivetrainHorizontalTranslation()),
            scale(operatorInterface->getDrivetrainRotationalTranslation()),
            0.0f
        );
        #else 
        chassis->setVelocityDrive(
            scale(operatorInterface->getDrivetrainVerticalTranslation()),
            scale(operatorInterface->getDrivetrainHorizontalTranslation()),
            scale(operatorInterface->getDrivetrainRotationalTranslation()),
            chassis->getYaw()
        );
        #endif

    }
// STEP 3 (Tank Drive): end function
    void ChassisDriveCommand::end(bool interrupted) {
        chassis->setVelocityDrive(0, 0, 0, 0);
    }
};  // namespace control::chassis