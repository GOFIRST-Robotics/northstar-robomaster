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

class ChassisDriveDistanceCommand : public tap::control::Command
{
public:
    ChassisDriveDistanceCommand(
        ChassisSubsystem *chassis,
        src::chassis::ChassisOdometry *chassisOdometry,
        float xDist,
        float yDist,
        float maxError);

    const char *getName() const override { return "Chassis drive dist"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    static constexpr float MAXIMUM_MPS = 1.0f;
    static constexpr float MINIMUM_MPS = 0.38f;

    src::chassis::ChassisSubsystem *chassis;
    src::chassis::ChassisOdometry *chassisOdometry;

    modm::Vector<float, 2> targetPosition;
    float maxError;
};
}  // namespace src::chassis