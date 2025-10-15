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
    static constexpr float MAX_CHASSIS_SPEED_MPS = 6.0f;

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
    src::chassis::ChassisSubsystem *chassis;
    src::chassis::ChassisOdometry *chassisOdometry;

    modm::Vector<float, 2> targetPosition;
    float maxError;

    using Pid = modm::Pid<float>;

    Pid xPid;
    Pid yPid;
};
}  // namespace src::chassis