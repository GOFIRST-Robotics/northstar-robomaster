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

class ChassisDriveDistanceCommand : public tap::control::Command
{
public:
    ChassisDriveDistanceCommand(
        ChassisSubsystem *chassis,
        src::control::ControlOperatorInterface *operatorInterface,
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

    src::control::ControlOperatorInterface *operatorInterface;

    float xDist;  // Forward
    float yDist;  // Sideways
    float maxError;

    float xDistanceCounter;
    float yDistanceCounter;

    uint32_t prevTime;

    using Pid = modm::Pid<float>;

    Pid xPid;
    Pid yPid;
};
}  // namespace src::chassis