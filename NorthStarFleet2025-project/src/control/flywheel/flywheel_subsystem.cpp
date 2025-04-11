#include "flywheel_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "flywheel_constants.hpp"


using namespace tap::motor;
using namespace tap::algorithms;

namespace src::control::flywheel
{
FlywheelSubsystem::FlywheelSubsystem(
    tap::Drivers *drivers,
    tap::motor::REVMotorId leftMotorId,
    tap::motor::REVMotorId rightMotorId,
    tap::motor::REVMotorId upMotorId,
    tap::can::CanBus canBus)
    : tap::control::Subsystem(drivers),
      leftWheel(drivers, leftMotorId, canBus, false, "Left Flywheel"),
      rightWheel(drivers, rightMotorId, canBus, false, "Right Flywheel"),
      upWheel(drivers, upMotorId, canBus, false, "Up Flywheel"),
      desiredLaunchSpeed(0),
      desiredRpmRamp(0){};

void FlywheelSubsystem::initialize()
{
    leftWheel.initialize();
    rightWheel.initialize();
    upWheel.initialize();
}

void FlywheelSubsystem::setDesiredLaunchSpeed(float speed)
{
    desiredLaunchSpeed = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
    desiredRpmRamp.setTarget(desiredLaunchSpeed);
}

float FlywheelSubsystem::getCurrentFlyWheelMotorRPM(tap::motor::RevMotor motor) const
{
    // return motor.getShaftRPM(); // TODO
    return 0.0f;
}

void FlywheelSubsystem::refresh()
{  // just use spark pid
   // uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
   // if (currTime == prevTime)
   // {
   //     return;
   // }
   // desiredRpmRamp.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
   // prevTime = currTime;

    // velocityPidLeftWheel.update(desiredRpmRamp.getValue() - leftWheel.getShaftRPM());
    // leftWheel.setDesiredOutput(static_cast<int32_t>(velocityPidLeftWheel.getValue()));
    // velocityPidRightWheel.update(desiredRpmRamp.getValue() - rightWheel.getShaftRPM());
    // rightWheel.setDesiredOutput(static_cast<int32_t>(velocityPidRightWheel.getValue()));
    // velocityPidUpWheel.update(desiredRpmRamp.getValue() - upWheel.getShaftRPM());
    // upWheel.setDesiredOutput(static_cast<int32_t>(velocityPidUpWheel.getValue()));
}
}  // namespace src::control::flywheel