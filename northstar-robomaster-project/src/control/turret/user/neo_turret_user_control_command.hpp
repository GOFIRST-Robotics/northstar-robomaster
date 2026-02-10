#ifndef NEO_TURRET_USER_CONTROL_COMMAND_HPP_
#define NEO_TURRET_USER_CONTROL_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "../algorithms/turret_controller_interface.hpp"
#include "control/turret/rev_turret_subsystem.hpp"

namespace src
{
class Drivers;

namespace control
{
class ControlOperatorInterface;
}
}  // namespace src

namespace src::control::turret::user
{
/**
 * Command that takes user input from the `ControlOperatorInterface` to control the pitch and yaw
 * axis of some turret using some passed in yaw and pitch controller upon construction.
 */
class NeoTurretUserControlCommand : public tap::control::Command
{
public:
    /**
     * @param[in] drivers Pointer to a global drivers object.
     * @param[in] turretSubsystem Pointer to the sentry turret to control.
     * @param[in] yawController Pointer to a yaw controller that will be used to control the yaw
     * axis of the turret.
     * @param[in] pitchController Pointer to a pitch controller that will be used to control the
     * pitch axis of the turret.
     * @param[in] userYawInputScalar Value to scale the user input from `ControlOperatorInterface`
     * by. Basically mouse sensitivity.
     * @param[in] userPitchInputScalar See userYawInputScalar.
     */
    NeoTurretUserControlCommand(
        tap::Drivers *drivers,
        ControlOperatorInterface &controlOperatorInterface,
        RevTurretSubsystem *turretSubsystem,
        algorithms::TurretYawControllerInterface *yawController,
        algorithms::TurretPitchControllerInterface *pitchController,
        float userYawInputScalar,
        float userPitchInputScalar,
        uint8_t turretID = 0);

    bool isReady() override;

    const char *getName() const override { return "User turret control"; }

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

private:
    tap::Drivers *drivers;
    ControlOperatorInterface &controlOperatorInterface;
    RevTurretSubsystem *turretSubsystem;

    uint32_t prevTime = 0;

    algorithms::TurretYawControllerInterface *yawController;
    algorithms::TurretPitchControllerInterface *pitchController;

    const float userYawInputScalar;
    const float userPitchInputScalar;

    const uint8_t turretID;
};
}  // namespace src::control::turret::user

#endif  // TURRET_USER_CONTROL_COMMAND_HPP_
