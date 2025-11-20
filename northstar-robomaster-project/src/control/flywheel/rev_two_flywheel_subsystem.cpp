#include "rev_two_flywheel_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "control/flywheel/flywheel_constants.hpp"

using namespace tap::motor;
using namespace tap::algorithms;
using namespace src::flywheel;

namespace src::control::flywheel
{
RevTwoFlywheelSubsystem::RevTwoFlywheelSubsystem(
    tap::Drivers *drivers,
    tap::motor::REVMotorId leftMotorId,
    tap::motor::REVMotorId rightMotorId,
    tap::can::CanBus canBus)
    : TwoFlywheelSubsystem(drivers),
      leftWheel(
          drivers,
          leftMotorId,
          canBus,
          RevMotor::ControlMode::VELOCITY,
          false,
          "Left Flywheel"),
      rightWheel(
          drivers,
          rightMotorId,
          canBus,
          RevMotor::ControlMode::VELOCITY,
          true,
          "Right Flywheel"),
      desiredLaunchSpeedLeft(0),
      desiredLaunchSpeedRight(0),
      desiredRpmRampLeft(0),
      desiredRpmRampRight(0),
      pidConfig(
          0,
          FLYWHEEL_PID_KP_REV,
          FLYWHEEL_PID_KI_REV,
          FLYWHEEL_PID_KD_REV,
          FLYWHEEL_PID_KF_REV,
          0,
          0,
          FLYWHEEL_PID_K_MIN_OUT_REV,
          FLYWHEEL_PID_K_MAX_OUT_REV)
{
}

void RevTwoFlywheelSubsystem::initialize()
{
    leftWheel.initialize();
    rightWheel.initialize();

    leftWheel.setMotorPID(pidConfig);
    rightWheel.setMotorPID(pidConfig);
}

/**
 * using the set spin sets a desired rpm for the flywheels with the up wheel scaled by the spin
 * @param[in] speed in meters per second
 */
void RevTwoFlywheelSubsystem::setDesiredLaunchSpeed(float speed)
{
    desiredLaunchSpeedLeft = speed;
    desiredLaunchSpeedRight = speed;

    desiredRpmRampLeft.setTarget(limitVal(
        launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft),
        0.0f,
        MAX_DESIRED_LAUNCH_SPEED_RPM));

    desiredRpmRampRight.setTarget(limitVal(
        launchSpeedToFlywheelRpm(desiredLaunchSpeedRight),
        0.0f,
        MAX_DESIRED_LAUNCH_SPEED_RPM));
}

float RevTwoFlywheelSubsystem::launchSpeedToFlywheelRpm(float launchSpeed) const
{
    return launchSpeedLinearInterpolator.interpolate(launchSpeed);
}

float debugLeft = 0;
float debugRight = 0;
float debugUp = 0;
float debugLeftD = 0;
float debugRightD = 0;
float debugUpD = 0;

void RevTwoFlywheelSubsystem::refresh()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    if (currTime == prevTime)
    {
        return;
    }

    desiredRpmRampLeft.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    desiredRpmRampRight.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    prevTime = currTime;

    leftWheel.setControlValue(desiredRpmRampLeft.getValue());
    rightWheel.setControlValue(desiredRpmRampRight.getValue());

    debugLeft = leftWheel.getEncoder()->getVelocity() * 60 / (2 * M_PI);
    debugRight = rightWheel.getEncoder()->getVelocity() * 60 / (2 * M_PI);

    debugLeftD = desiredRpmRampLeft.getValue();
    debugRightD = desiredRpmRampRight.getValue();
}
}  // namespace src::control::flywheel