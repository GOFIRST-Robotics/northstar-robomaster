#ifndef SENTRY_SCAN_COMMAND_HPP
#define SENTRY_SCAN_COMMAND_HPP

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "control/turret/algorithms/turret_controller_interface.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace src::control::turret::cv
{
class SentryScanCommand : public tap::control::Command
{
public:
    SentryScanCommand(
        tap::Drivers *drivers,
        TurretSubsystem *turretSubsystem,
        algorithms::TurretYawControllerInterface *yawController,
        algorithms::TurretPitchControllerInterface *pitchController,
        float DELTA_MAX,
        float MAX_ERROR,
        float ROT_SPEED);

    bool isReady() override;

    const char *getName() const override { return "Sentry scan"; }

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

private:
    tap::Drivers *drivers;
    TurretSubsystem *turretSubsystem;

    uint32_t prevTime = 0;

    algorithms::TurretYawControllerInterface *yawController;
    algorithms::TurretPitchControllerInterface *pitchController;

    float comp = 0;
    float yawSetpointTop = 0;
    float DELTA_MAX;
    float MAX_ERROR;
    float ROT_SPEED;
};
}  // namespace src::control::turret::cv

#endif  // SENTRY_TURRET_USER_CONTROL_COMMAND_HPP_