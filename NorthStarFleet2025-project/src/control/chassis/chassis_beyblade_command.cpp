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

    double prevTime;    

    void ChassisBeybladeCommand::execute() {
        double currTime = tap::arch::clock::getTimeMilliseconds();
        double dt = currTime - prevTime;
        prevTime = currTime;

        chassis->updateBeyBladeRotationSpeed(1, dt);
        auto scale = [](float raw) -> float {
            return limitVal(raw, -1.0f, 1.0f) * MAX_CHASSIS_SPEED_MPS;
        };
        chassis->setVelocityTurretDrive(
            scale(operatorInterface->getDrivetrainVerticalTranslation()),
            -scale(operatorInterface->getDrivetrainHorizontalTranslation()),
            scale(chassis->getBeyBlade())
        );
    }
// STEP 3 (Tank Drive): end function
    void ChassisBeybladeCommand::end(bool interrupted) {
        chassis->updateBeyBladeRotationSpeed(0, 0);
    }
};  // namespace control::chassis