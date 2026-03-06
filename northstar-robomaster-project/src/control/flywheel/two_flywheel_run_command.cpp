#include "two_flywheel_run_command.hpp"

namespace src::control::flywheel
{
TwoFlywheelRunCommand::TwoFlywheelRunCommand(
    TwoFlywheelSubsystem *flywheel,
    float launchSpeed = 20.0f)
    : flywheel(flywheel),
      launchSpeed(launchSpeed)

{
    addSubsystemRequirement(flywheel);
}

void TwoFlywheelRunCommand::initialize() { flywheel->setDesiredLaunchSpeed(launchSpeed); }

void TwoFlywheelRunCommand::end(bool interrupted) { flywheel->setDesiredLaunchSpeed(0); }

}  // namespace src::control::flywheel