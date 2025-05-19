#include "hero_agitator_subsystem.hpp"

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/drivers.hpp"

using namespace src::control::agitator::constants;

namespace src::agitator
{
HeroAgitatorSubsystem::HeroAgitatorSubsystem(
    tap::Drivers* drivers,
    const HeroAgitatorSubsystemConfig& config,
    const tap::algorithms::SmoothPidConfig& pidConfig)
    : tap::control::Subsystem(drivers),
      config(config),
      agitatorServo(
          drivers,
          config.agitatorServoId,
          config.maximumPwm,
          config.minimumPwm,
          config.pwmRampSpeed),
      agitatorMotor(
          drivers,
          config.agitatorMotorId,
          config.canBus,
          config.agitatorMotorInverted,
          "Agitator"),
      limitSwitch(&drivers->digital, config.pin, config.limitSwitchInverted),
      pid(pidConfig)

{
    agitatorServo.setTargetPwm(agitatorServo.getMinPWM());
}

void HeroAgitatorSubsystem::initialize() { agitatorMotor.initialize(); }

void HeroAgitatorSubsystem::shoot() { agitatorServo.setTargetPwm(agitatorServo.getMaxPWM()); }

void HeroAgitatorSubsystem::reload()
{
    agitatorServo.setTargetPwm(agitatorServo.getMinPWM());
    velocitySetpoint = M_TWOPI * 1.5;
    loaded = false;
    reloadTimeout.restart(config.reloadTimeout);
}

void HeroAgitatorSubsystem::refresh()
{
    if (!loaded && !limitSwitch.getLimitSwitchDepressed())
    {
        loaded = true;
    }
    if (reloadTimeout.isExpired() || loaded && limitSwitch.getLimitSwitchDepressed())
    {
        velocitySetpoint = 0;
    }
    agitatorServo.updateSendPwmRamp();
    runVelocityPidControl();
}
float HeroAgitatorSubsystem::getUncalibratedAgitatorAngle() const
{
    return (2.0f * M_PI / tap::motor::DjiMotor::ENC_RESOLUTION) *
           agitatorMotor.getEncoderUnwrapped() / config.agitatorGearRatio;
}

void HeroAgitatorSubsystem::runVelocityPidControl()
{
    const uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
    const uint32_t dt = curTime - prevTime;
    prevTime = curTime;

    const float velocityError = velocitySetpoint - getVelocity();

    pid.runControllerDerivateError(velocityError, dt);

    agitatorMotor.setDesiredOutput(pid.getOutput() + velocitySetpoint);
}

}  // namespace src::agitator