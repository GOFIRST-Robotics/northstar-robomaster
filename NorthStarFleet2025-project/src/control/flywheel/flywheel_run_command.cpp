#include "flywheel_run_command.hpp"

namespace src::control::flywheel
{
FlywheelRunCommand::FlywheelRunCommand(FlywheelSubsystem *flywheel) : flywheel(flywheel)
{
    addSubsystemRequirement(flywheel);
}

void FlywheelRunCommand::initialize()
{
    flywheel->setDesiredLaunchSpeedLeft(.5);
    flywheel->setDesiredLaunchSpeedRight(.5);
    flywheel->setDesiredLaunchSpeedUp(.5);
}

void FlywheelRunCommand::end(bool interrupted)
{
    flywheel->setDesiredLaunchSpeedLeft(0);
    flywheel->setDesiredLaunchSpeedRight(0);
    flywheel->setDesiredLaunchSpeedUp(0);
}

}  // namespace src::control::flywheel