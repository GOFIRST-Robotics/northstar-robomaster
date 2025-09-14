#include "hopper_toggle_command.hpp"

namespace src::control::hopper
{
HopperToggleCommand::HopperToggleCommand(HopperSubsystem* hopper)
    : tap::control::Command(),
      hopper(hopper),
      startTime(0)
{
    addSubsystemRequirement(hopper);
}

void HopperToggleCommand::initialize() { hopper->open(); }

// set the hopper servo to the open position
void HopperToggleCommand::execute() {}

// set the hopper servo to the close position
void HopperToggleCommand::end(bool) { hopper->close(); }

bool HopperToggleCommand::isReady() { return true; }

bool HopperToggleCommand::isFinished() const { return false; }

}  // namespace src::control::hopper
