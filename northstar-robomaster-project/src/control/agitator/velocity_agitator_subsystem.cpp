#include "velocity_agitator_subsystem.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "modm/math/filter/pid.hpp"

using namespace tap::motor;

namespace src::agitator
{
VelocityAgitatorSubsystem::VelocityAgitatorSubsystem(
    tap::Drivers* drivers,
    const tap::algorithms::SmoothPidConfig& pidParams,
    const VelocityAgitatorSubsystemConfig& agitatorSubsystemConfig)
    : tap::control::Subsystem(drivers),
      config(agitatorSubsystemConfig),
      velocityPid(pidParams),
      jamChecker(this, config.jammingVelocityDifference, config.jammingTime),
      agitatorMotor(
          drivers,
          config.agitatorMotorId,
          config.agitatorCanBusId,
          config.isAgitatorInverted,
          "agitator motor",
          false,
          config.gearRatio)
{
    assert(config.jammingVelocityDifference >= 0);
}

void VelocityAgitatorSubsystem::initialize() { agitatorMotor.initialize(); }

void VelocityAgitatorSubsystem::refresh()
{
    if (!isOnline())
    {
        agitatorIsCalibrated = false;
    }

    if (!agitatorIsCalibrated)
    {
        if (!calibrateHere())
        {
            return;
        }
    }

    runVelocityPidControl();

    if (jamChecker.check())
    {
        subsystemJamStatus = true;
    }
}

bool VelocityAgitatorSubsystem::calibrateHere()
{
    if (!isOnline())
    {
        return false;
    }
    agitatorMotor.getEncoder()->resetEncoderValue();
    agitatorIsCalibrated = true;
    velocitySetpoint = 0.0f;
    clearJam();
    return true;
}

float VelocityAgitatorSubsystem::getCurrentValueIntegral() const
{
    if (!agitatorIsCalibrated)
    {
        return 0.0f;
    }
    return agitatorMotor.getEncoder()->getPosition().getUnwrappedValue();
}

void VelocityAgitatorSubsystem::runVelocityPidControl()
{
    const uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
    const uint32_t dt = curTime - prevTime;
    prevTime = curTime;

    const float velocityError = velocitySetpoint - getCurrentValue();

    velocityPid.runControllerDerivateError(velocityError, dt);

    agitatorMotor.setDesiredOutput(
        velocityPid.getOutput() + velocitySetpoint * config.velocityPIDFeedForwardGain);
}

void VelocityAgitatorSubsystem::setSetpoint(float velocity)
{
    if (agitatorMotor.isMotorOnline())
    {
        velocitySetpoint = velocity;
    }
}
}  // namespace src::agitator