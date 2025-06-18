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
      pwm(config.reloadPwm),
      //   agitatorServo(
      //       drivers,
      //       config.agitatorServoId,
      //       config.maximumPwm,
      //       config.minimumPwm,
      //       config.pwmRampSpeed),
      agitatorMotor(
          drivers,
          config.agitatorMotorId,
          config.canBus,
          config.agitatorMotorInverted,
          "Agitator",
          false,
          config.agitatorGearRatio),
      limitSwitch(&drivers->digital, config.pin, config.limitSwitchInverted),
      pid(pidConfig)

{
    // agitatorServo.setTargetPwm(agitatorServo.getMinPWM());
}

void HeroAgitatorSubsystem::initialize()
{
    agitatorMotor.initialize();
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER1, 330);  // Timer 1 for C1 Pin
}

void HeroAgitatorSubsystem::shoot()
{
    pwm = config.shootPwm;
    agitatorTimeout.restart(config.reloadTimeout);
    loaded = false;
}

void HeroAgitatorSubsystem::setPWM(float dutyCycle)
{
    drivers->pwm.write(dutyCycle, tap::gpio::Pwm::Pin::C1);
}

void HeroAgitatorSubsystem::reload()
{
    pwm = config.reloadPwm;
    velocitySetpoint = M_TWOPI * 0.5f;
    reloading = true;
}

void HeroAgitatorSubsystem::refresh()
{
    if (reloading && !loaded && !limitSwitch.getLimitSwitchDepressed())
    {
        loaded = true;
        reloading = false;
    }
    if ((agitatorTimeout.isExpired() || loaded) && limitSwitch.getLimitSwitchDepressed())
    {
        pwm = 0.4f;
        velocitySetpoint = 0;
    }
    setPWM(pwm);
    // agitatorServo.updateSendPwmRamp();
    runVelocityPidControl();
}
float HeroAgitatorSubsystem::getUncalibratedAgitatorAngle() const
{
    return agitatorMotor.getEncoder()->getPosition().getUnwrappedValue();
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