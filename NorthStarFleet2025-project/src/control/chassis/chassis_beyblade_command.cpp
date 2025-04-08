#include "chassis_beyblade_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
// STEP 1 (Tank Drive): Constructor
ChassisBeybladeCommand::ChassisBeybladeCommand(ChassisSubsystem* chassis, src::control::ControlOperatorInterface* operatorInterface) :
    chassis(chassis),
    operatorInterface(operatorInterface)
    {
        addSubsystemRequirement(chassis);
    }
// STEP 2 (Tank Drive): execute function

    void ChassisBeybladeCommand::initialize() {
        prevTime = tap::arch::clock::getTimeMilliseconds();
    }   

    void ChassisBeybladeCommand::execute() {
        uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
        uint32_t dt = currTime - prevTime;
        prevTime = currTime;

        auto scale = [](float raw) -> float {
            return limitVal(raw, -1.0f, 1.0f) * MAX_CHASSIS_SPEED_MPS;
        };
        chassis->setVelocityBeyBladeDrive(
            scale(operatorInterface->getDrivetrainVerticalTranslation()),
            -scale(operatorInterface->getDrivetrainHorizontalTranslation()),
            1,
            dt
        );
    }
// STEP 3 (Tank Drive): end function
    void ChassisBeybladeCommand::end(bool interrupted) {
        chassis->setVelocityBeyBladeDrive(0, 0, 0, 0);
    }
};  // namespace control::chassis