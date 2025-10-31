#pragma once

#include "tap/control/command.hpp"

#include "control/chassis/constants/chassis_constants.hpp"

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

class ChassisFieldCommand : public tap::control::Command
{
public:
    ChassisFieldCommand(
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
};
}  // namespace src::chassis