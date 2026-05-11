#include "chassis_sprint_command.hpp"

#include "chassis_subsystem.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisSprintCommand::ChassisSprintCommand(ChassisSubsystem* chassis) : chassis(chassis)
{
    addSubsystemRequirement(chassis);
}

void ChassisSprintCommand::initialize() { chassis->setIsSprinting(true); }

void ChassisSprintCommand::end([[maybe_unused]] bool interrupted)
{
    chassis->setIsSprinting(false);
}
};  // namespace src::chassis