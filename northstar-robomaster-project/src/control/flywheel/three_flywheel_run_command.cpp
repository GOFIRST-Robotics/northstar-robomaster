#ifndef TARGET_HERO

#include "three_flywheel_run_command.hpp"

namespace src::control::flywheel
{
ThreeFlywheelRunCommand::ThreeFlywheelRunCommand(
    ThreeFlywheelSubsystem *flywheel,
    float launchSpeed = 20.0f,
    float spin = 100.0f)
    : flywheel(flywheel),
      launchSpeed(launchSpeed),
      spin(spin)

{
    addSubsystemRequirement(flywheel);
}

void ThreeFlywheelRunCommand::initialize()
{
    flywheel->setDesiredSpin(spin);
    flywheel->setDesiredLaunchSpeed(launchSpeed);
}

void ThreeFlywheelRunCommand::end(bool interrupted) { flywheel->setDesiredLaunchSpeed(0); }

}  // namespace src::control::flywheel

#endif  // TARGET_HERO