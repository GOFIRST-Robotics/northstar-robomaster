#ifndef HOPPER_TOGGLE_COMMAND_HPP_
#define HOPPER_TOGGLE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "control/hopper/hopper_subsystem.hpp"

namespace src::control::hopper
{
class HopperToggleCommand : public tap::control::Command
{
public:
    HopperToggleCommand(HopperSubsystem* hopper);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "hopper toggle"; }

private:
    HopperSubsystem* hopper;

    uint32_t startTime = 0;
};

}  // namespace src::control::hopper

#endif  // HERO_AGITATOR_SHOOT_COMMAND_HPP_
