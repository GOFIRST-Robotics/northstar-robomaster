#pragma once

#include "tap/control/command.hpp"

#include "control/chassis/constants/chassis_constants.hpp"
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
    ChassisOrientDriveCommand(
        ChassisSubsystem *chassis,
        src::control::ControlOperatorInterface *operatorInterface);

    const char *getName() const override { return "Chassis tank drive"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    src::chassis::ChassisSubsystem *chassis;

    src::control::ControlOperatorInterface *operatorInterface;

    float rotationalValue;
};
}  // namespace src::chassis