#ifndef FLYWHEEL_RUN_COMMAND
#define FLYWHEEL_RUN_COMMAND

#include "tap/control/command.hpp"

#include "control/flywheel/three_flywheel_subsystem.hpp"

namespace src
{
class Drivers;

namespace control
{
class ControlOperatorInterface;
}
}  // namespace src

namespace src::control::flywheel
{
class ThreeFlywheelRunCommand : public tap::control::Command
{
public:
    ThreeFlywheelRunCommand(ThreeFlywheelSubsystem *flywheel, float launchSpeed, float spin);

    const char *getName() const override { return "Flywheel Run Command"; }

    void initialize() override;

    void execute() override {}

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    ThreeFlywheelSubsystem *flywheel;

    float launchSpeed;
    float spin;
};
}  // namespace src::control::flywheel
#endif  // FLYWHEEL_RUN_COMMAND
