#pragma once

#include "tap/control/command.hpp"

#include "modm/math/filter/pid.hpp"

namespace src
{
class Drivers;

namespace control
{
class ControlOperatorInterface;
}
}  // namespace src

namespace src::chassis
{
class ChassisSubsystem;

class ChassisOrientDriveCommand : public tap::control::Command
{
public:
    static constexpr float MAX_CHASSIS_SPEED_MPS = 7.0f;

    ChassisOrientDriveCommand(
        ChassisSubsystem *chassis,
        src::control::ControlOperatorInterface *operatorInterface);

    const char *getName() const override { return "Chassis tank drive"; }

    void initialize() override {}

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    src::chassis::ChassisSubsystem *chassis;

    src::control::ControlOperatorInterface *operatorInterface;

    modm::Pid<float> orientPid;

    float angleOffset;
};
}  // namespace src::chassis