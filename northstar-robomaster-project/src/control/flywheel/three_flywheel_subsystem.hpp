#ifndef THREE_FLYWHEEL_SUBSYSTEM_HPP_
#define THREE_FLYWHEEL_SUBSYSTEM_HPP_

#include <modm/container/pair.hpp>

#include "tap/control/subsystem.hpp"

#include "control/flywheel/flywheel_constants.hpp"

#include "flywheel_interface.hpp"

namespace src::control::flywheel
{
class ThreeFlywheelSubsystem : public tap::control::Subsystem, public FlywheelInterface
{
public:
    explicit ThreeFlywheelSubsystem(
        tap::Drivers *drivers,
        std::array<std::array<modm::Pair<float, float>, 4>, SPIN_COUNT> spinToRPMMap)
        : tap::control::Subsystem::Subsystem(drivers),
          spinToRPMMap(spinToRPMMap)
    {
    }

    virtual void setDesiredSpin(u_int16_t spin) = 0;

    virtual float getDesiredSpin() const = 0;

protected:
    Spin desiredSpin = SPIN_100;
    u_int16_t desiredSpinValue = 100;  // percent of spin

    std::array<std::array<modm::Pair<float, float>, 4>, SPIN_COUNT> spinToRPMMap;
};

}  // namespace src::control::flywheel

#endif  // THREE_FLYWHEEL_SUBSYSTEM_HPP_