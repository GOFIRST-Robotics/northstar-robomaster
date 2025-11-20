#ifndef TWO_FLYWHEEL_RUN_COMMAND
#define TWO_FLYWHEEL_RUN_COMMAND

#include "tap/control/command.hpp"

#include "control/flywheel/two_flywheel_subsystem.hpp"

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
class TwoFlywheelRunCommand : public tap::control::Command
{
public:
    TwoFlywheelRunCommand(TwoFlywheelSubsystem *flywheel, float launchSpeed);

    const char *getName() const override { return "Flywheel Run Command"; }

    void initialize() override;

    void execute() override {}

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    TwoFlywheelSubsystem *flywheel;

    float launchSpeed;
};
}  // namespace src::control::flywheel
#endif  // FLYWHEEL_RUN_COMMAND
