#ifndef fire_rate_cycler_mapping_HPP_
#define fire_rate_cycler_mapping_HPP_

#include <optional>

#include "tap/control/hold_repeat_command_mapping.hpp"

#include "control/agitator/constant_velocity_agitator_command.hpp"

#include "manual_fire_rate_reselection_manager.hpp"

namespace src::control::agitator
{
/**
 * Class that stores and allows the user to set some LaunchMode. Possible launch modes include
 * single, 10 Hz, or 20 Hz full auto mode.
 *
 * This object is a HoldRepeatCommandMapping. An instance of this object should be added to the
 * global CommandMapper to use it. This object contains a launch Command that it will schedule. How
 * often/when the launch Command will be schedule is based on the launch mode. This object controls
 * the fire rate and firing frequency of the launch Command based on the launch mode.
 *
 * If vision is running, the fire rate should not be limited and the launcher should be in full auto
 * mode, so this object checks the launchMode of a CvOnTargetGovernor when setting the fire rate.
 */
class FireRateCyclerMapping : public tap::control::HoldRepeatCommandMapping
{
public:
    /**
     * State of the shooting mechanism, how many times the associated command mapping should
     * reschedule the command when the mapping is met.
     */
    enum LaunchMode : uint8_t
    {
        SINGLE = 0,
        LIMITED_10HZ,
        LIMITED_20HZ,
        FULL_AUTO,
        NUM_SHOOTER_STATES,
    };

    /**
     * @param[in] drivers Reference to global drivers object.
     * @param[in] launchCommand Command that when scheduled launches a single projectile.
     * @param[in] rms The remote mapping that controls when the launch command should be scheduled.
     * @param[in] fireRateReselectionManager An optional argument, the fire rate reselection manager
     * that controls the fire rate of the launch command. If provided, the manager's fire rate is
     * updated based on the current LaunchMode.
     */
    FireRateCyclerMapping(
        tap::Drivers &drivers,
        tap::control::Command &launchCommand,
        const tap::control::RemoteMapState &rms,
        std::optional<ManualFireRateReselectionManager *> fireRateReselectionManager,
        std::optional<ConstantVelocityAgitatorCommand *> command = std::nullopt);

    void setShooterState(LaunchMode mode)
    {
        if (mode < NUM_SHOOTER_STATES)
        {
            this->launchMode = mode;
        }
    }

    LaunchMode getLaunchMode() const { return launchMode; }

    void executeCommandMapping(const tap::control::RemoteMapState &currState);

private:
    std::optional<ManualFireRateReselectionManager *> fireRateReselectionManager;

    LaunchMode launchMode = SINGLE;
    std::optional<ConstantVelocityAgitatorCommand *> command;
};

}  // namespace src::control::agitator

#endif  // MULTI_SHOT_CV_COMMAND_MAPPING_HPP_
