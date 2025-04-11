#include "hero_flywheel_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "hero_flywheel_constants.hpp"

using namespace tap::motor;
using namespace tap::algorithms;

namespace src::control::flywheel
{
HeroFlywheelSubsystem::HeroFlywheelSubsystem(
    tap::Drivers *drivers,
    tap::motor::MotorId leftMotorId,
    tap::motor::MotorId rightMotorId,
    tap::motor::MotorId upMotorId,
    tap::can::CanBus canBus)
    : tap::control::Subsystem(drivers),
      velocityPidLeftWheel(
          FLYWHEEL_PID_KP,
          FLYWHEEL_PID_KI,
          FLYWHEEL_PID_KD,
          FLYWHEEL_PID_MAX_ERROR_SUM,
          FLYWHEEL_PID_MAX_OUTPUT),
      velocityPidRightWheel(
          FLYWHEEL_PID_KP,
          FLYWHEEL_PID_KI,
          FLYWHEEL_PID_KD,
          FLYWHEEL_PID_MAX_ERROR_SUM,
          FLYWHEEL_PID_MAX_OUTPUT),
      velocityPidUpWheel(
          FLYWHEEL_PID_KP,
          FLYWHEEL_PID_KI,
          FLYWHEEL_PID_KD,
          FLYWHEEL_PID_MAX_ERROR_SUM,
          FLYWHEEL_PID_MAX_OUTPUT),
      leftWheel(drivers, leftMotorId, canBus, true, "Left Flywheel"),
      rightWheel(drivers, rightMotorId, canBus, false, "Right Flywheel"),
      upWheel(drivers, upMotorId, canBus, false, "Up Flywheel"),
      desiredLaunchSpeed(0),
      desiredRpmRamp(0){};

void HeroFlywheelSubsystem::initialize()
{
    leftWheel.initialize();
    rightWheel.initialize();
    upWheel.initialize();
    setDesiredLaunchSpeed(500);
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void HeroFlywheelSubsystem::setDesiredLaunchSpeed(float speed)
{
    desiredLaunchSpeed = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
    desiredRpmRamp.setTarget(desiredLaunchSpeed);
}

float HeroFlywheelSubsystem::getCurrentFlyWheelMotorRPM(tap::motor::DjiMotor motor) const
{
    // return motor.getShaftRPM(); // TODO
    return 0.0f;
}

void HeroFlywheelSubsystem::refresh()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    if (currTime == prevTime)
    {
        return;
    }
    desiredRpmRamp.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    prevTime = currTime;

    velocityPidLeftWheel.update(desiredRpmRamp.getValue() - leftWheel.getShaftRPM());
    leftWheel.setDesiredOutput(static_cast<int32_t>(velocityPidLeftWheel.getValue()));
    velocityPidRightWheel.update(desiredRpmRamp.getValue() - rightWheel.getShaftRPM());
    rightWheel.setDesiredOutput(static_cast<int32_t>(velocityPidRightWheel.getValue()));
    velocityPidUpWheel.update(desiredRpmRamp.getValue() - upWheel.getShaftRPM());
    upWheel.setDesiredOutput(static_cast<int32_t>(velocityPidUpWheel.getValue()));
}
}  // namespace src::control::flywheel