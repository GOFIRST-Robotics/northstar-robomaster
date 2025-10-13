#include "fire_rate_cycler_mapping.hpp"

namespace src::control::agitator
{
FireRateCyclerMapping::FireRateCyclerMapping(
    tap::Drivers &drivers,
    tap::control::Command &launchCommand,
    const tap::control::RemoteMapState &rms,
    std::optional<ManualFireRateReselectionManager *> fireRateReselectionManager,
    std::optional<ConstantVelocityAgitatorCommand *> command)
    : tap::control::HoldRepeatCommandMapping(&drivers, {&launchCommand}, rms, false),
      fireRateReselectionManager(fireRateReselectionManager),
      command(command)
{
}

void FireRateCyclerMapping::executeCommandMapping(const tap::control::RemoteMapState &currState)
{
    int timesToReschedule = 0;

    float fireRate = 0.0f;

    bool enableConstantRotation = false;

    switch (launchMode)
    {
        case SINGLE:
            timesToReschedule = 1;
            fireRate = ManualFireRateReselectionManager::MAX_FIRERATE_RPS;
            break;
        case LIMITED_10HZ:
            timesToReschedule = -1;
            fireRate = 10;
            break;
        case LIMITED_20HZ:
            timesToReschedule = -1;
            fireRate = 20;
            break;
        case FULL_AUTO:
            timesToReschedule = -1;
            fireRate = ManualFireRateReselectionManager::MAX_FIRERATE_RPS;
            enableConstantRotation = true;
            break;
        default:
            assert(false);
            break;
    }

    if (command.has_value())
    {
        command.value()->enableConstantRotation(enableConstantRotation);
    }

    if (fireRateReselectionManager.has_value())
    {
        fireRateReselectionManager.value()->setFireRate(fireRate);
    }

    setMaxTimesToSchedule(timesToReschedule);

    tap::control::HoldRepeatCommandMapping::executeCommandMapping(currState);

    if (!held)
    {
        if (command.has_value())
        {
            command.value()->enableConstantRotation(false);
        }
    }
}

}  // namespace src::control::agitator
