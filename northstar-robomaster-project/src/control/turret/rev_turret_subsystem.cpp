#include "rev_turret_subsystem.hpp"

#include <algorithm>
#include <cfloat>
#include <random>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/errors/create_errors.hpp"

using namespace tap::motor;
using namespace tap::algorithms;

namespace src::control::turret
{
RevTurretSubsystem::RevTurretSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface* pitchMotor,
    tap::motor::RevMotor* yawMotor1,
    tap::motor::RevMotor* yawMotor2,
    const TurretMotorConfig& pitchMotorConfig,
    const TurretMotorConfig& yawMotorConfig)
    : tap::control::Subsystem(drivers),
      pitchMotor(pitchMotor, pitchMotorConfig),
      yawMotor(yawMotor1, yawMotor2, yawMotorConfig)
{
    assert(drivers != nullptr);
    assert(pitchMotor != nullptr);
    assert(yawMotor1 != nullptr);
    assert(yawMotor2 != nullptr);
}

void RevTurretSubsystem::initialize()
{
    yawMotor.initialize();
    pitchMotor.initialize();
}

void RevTurretSubsystem::refresh()
{
    yawMotor.updateMotorAngle();
    pitchMotor.updateMotorAngle();
}
}  // namespace src::control::turret
