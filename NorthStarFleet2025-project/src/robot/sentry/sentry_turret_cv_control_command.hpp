#ifndef SENTRY_TURRET_CV_CONTROL_COMMAND_HPP_
#define SENTRY_TURRET_CV_CONTROL_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "communication/serial/vision_comms.hpp"
#include "control/turret/CV/turret_cv_control_command_template.hpp"
#include "control/turret/algorithms/turret_controller_interface.hpp"
#include "robot/control_operator_interface.hpp"
#include "robot/sentry/sentry_turret_subsystem.hpp"

namespace src::control::turret::cv
{
/**
 * Command that takes user input from the `ControlOperatorInterface` to control the pitch and yaw
 * axis of some turret using some passed in yaw and pitch controller upon construction.
 */
class SentryTurretCVControlCommand : public TurretCVControlCommandTemplate
{
public:
    /**
     * @param[in] drivers Pointer to a global drivers object.
     * @param[in] turretSubsystem Pointer to the sentry turret to control.
     * @param[in] yawControllerTop Pointer to a yaw controller that will be used to control the yaw
     * axis of the turret.
     * @param[in] pitchControllerTop Pointer to a pitch controller that will be used to control the
     * pitch axis of the turret.
     ** @param[in] yawControllerBottom Pointer to a yaw controller that will be used to control
     the yaw
     * axis of the turret.
     * @param[in] pitchControllerBottom Pointer to a pitch controller that will be used to control
     the
     * pitch axis of the turret.
     * @param[in] userYawInputScalar Value to scale the user input from `ControlOperatorInterface`
     * by. Basically mouse sensitivity.
     * @param[in] userPitchInputScalar See userYawInputScalar.
     */
    SentryTurretCVControlCommand(
        tap::Drivers *drivers,
        ControlOperatorInterface &controlOperatorInterface,
        src::serial::VisionComms &visionComms,
        SentryTurretSubsystem *turretSubsystem,
        algorithms::TurretYawControllerInterface *yawControllerBottom,
        algorithms::TurretPitchControllerInterface *pitchControllerBottom,
        algorithms::TurretYawControllerInterface *yawControllerTop,
        algorithms::TurretPitchControllerInterface *pitchControllerTop,
        float userYawInputScalar,
        float userPitchInputScalar,
        float MAX_ERROR,
        float DELTA_MAX);

    bool isReady() override;

    const char *getName() const override { return "Sentry cv turret control"; }

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool interrupted) override;

    bool isAimingWithinLaunchingTolerance(uint8_t turretID) const
    {
        return turretID == 0 ? bottomWithinAimingTolerance : topWithinAimingTolerance;
    }

private:
    tap::Drivers *drivers;
    ControlOperatorInterface &controlOperatorInterface;
    src::serial::VisionComms &visionComms;
    SentryTurretSubsystem *turretSubsystem;

    uint32_t prevTime = 0;

    algorithms::TurretYawControllerInterface *yawControllerTop;
    algorithms::TurretPitchControllerInterface *pitchControllerTop;
    algorithms::TurretYawControllerInterface *yawControllerBottom;
    algorithms::TurretPitchControllerInterface *pitchControllerBottom;

    float bottomMeasurementOffset;
    float topMeasurementOffset;
    float bottomSetpointOffset;
    float comp = 0;
    float yawSetpointTop = 0;
    float DELTA_MAX;
    float MAX_ERROR;

    const float userYawInputScalar;
    const float userPitchInputScalar;

    float AIMING_TOLERANCE_YAW = .05;
    float AIMING_TOLERANCE_PITCH = .05;

    bool bottomWithinAimingTolerance = false;
    bool topWithinAimingTolerance = false;
};  // namespace src::control::turret::cv
}  // namespace src::control::turret::cv

#endif  // SENTRY_TURRET_CV_CONTROL_COMMAND_HPP_
