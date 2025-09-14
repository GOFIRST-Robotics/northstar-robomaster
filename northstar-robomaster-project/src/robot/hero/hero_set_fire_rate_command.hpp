#ifndef HERO_SET_FIRE_RATE_COMMAND_HPP_
#define HERO_SET_FIRE_RATE_COMMAND_HPP_

#include <optional>

#include "tap/control/command.hpp"

#include "control/agitator/manual_fire_rate_reselection_manager.hpp"

namespace src::agitator
{
class HeroSetFireRateCommand : public tap::control::Command
{
public:
    HeroSetFireRateCommand(
        tap::control::Subsystem *subsystem,
        src::control::agitator::ManualFireRateReselectionManager &fireRateReselectionManager,
        u_int8_t fireRate)
        : fireRateReselectionManager(fireRateReselectionManager),
          fireRate(fireRate)
    {
        addSubsystemRequirement(subsystem);
    }

    const char *getName() const override { return "Fire rate command"; }

    void initialize() override { fireRateReselectionManager.setFireRate(fireRate); }

    void execute() override {}

    void end(bool interrupted) override {}

    bool isFinished() const { return true; }

private:
    src::control::agitator::ManualFireRateReselectionManager &fireRateReselectionManager;
    u_int8_t fireRate = 0;
};
}  // namespace src::agitator

#endif
