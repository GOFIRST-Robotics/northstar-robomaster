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

class ChassisDriveDistanceCommand : public tap::control::Command
{
public:
    static constexpr float MAX_CHASSIS_SPEED_MPS = 6.0f;

    ChassisDriveDistanceCommand(
        ChassisSubsystem *chassis,
        src::control::ControlOperatorInterface *operatorInterface,
        float xDist,
        float yDist);

    const char *getName() const override { return "Chassis tank drive"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    src::chassis::ChassisSubsystem *chassis;

    src::control::ControlOperatorInterface *operatorInterface;

    float xDist;
    float yDist;

    float xDistanceCounter;
    float yDistanceCounter;

    uint32_t prevTime;

    using Pid = modm::Pid<float>;

    Pid xPid;
    Pid yPid;
};
}  // namespace src::chassis