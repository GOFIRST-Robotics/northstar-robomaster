#ifndef TWO_FLYWHEEL_RUN_RPM_COMMAND
#define TWO_FLYWHEEL_RUN_RPM_COMMAND

#include "tap/control/command.hpp"

#include "control/flywheel/two_flywheel_subsystem.hpp"

namespace src::control::flywheel
{
class TwoFlywheelRunRPMCommand : public tap::control::Command
{
public:
    TwoFlywheelRunRPMCommand(TwoFlywheelSubsystem *flywheel, float rpm);

    const char *getName() const override { return "Flywheel Run Command"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    TwoFlywheelSubsystem *flywheel;

    float rpm;
};
}  // namespace src::control::flywheel
#endif  // FLYWHEEL_RUN_COMMAND
