#ifndef TWO_FLYWHEEL_SUBSYSTEM_HPP_
#define TWO_FLYWHEEL_SUBSYSTEM_HPP_

#include <modm/container/pair.hpp>

#include "tap/control/subsystem.hpp"

#include "control/flywheel/flywheel_constants.hpp"

#include "flywheel_interface.hpp"

namespace src::control::flywheel
{
class TwoFlywheelSubsystem : public tap::control::Subsystem, public FlywheelInterface
{
public:
    explicit TwoFlywheelSubsystem(tap::Drivers *drivers)
        : tap::control::Subsystem::Subsystem(drivers),
          launchSpeedLinearInterpolator(MPS_TO_RPM, MODM_ARRAY_SIZE(MPS_TO_RPM))
    {
    }

protected:
    modm::interpolation::Linear<modm::Pair<float, float>> launchSpeedLinearInterpolator;
};

}  // namespace src::control::flywheel

#endif  // THREE_FLYWHEEL_SUBSYSTEM_HPP_