#pragma once

#include "tap/control/command.hpp"

#include "modm/math/filter/pid.hpp"

#include "chassis_subsystem.hpp"

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

class ChassisDriveToPointCommand : public tap::control::Command
{
public:
    ChassisDriveToPointCommand(
        ChassisSubsystem *chassis,
        src::chassis::ChassisOdometry *chassisOdometry,
        float xPosition,
        float yPosition,
        float maxError);

    const char *getName() const override { return "Chassis drive to point"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    src::chassis::ChassisSubsystem *chassis;
    src::chassis::ChassisOdometry *chassisOdometry;

    modm::Vector<float, 2> targetPosition;
    float maxError;
};
}  // namespace src::chassis