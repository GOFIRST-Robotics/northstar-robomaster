#ifndef TARGET_HERO

#include "flywheel_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "control/flywheel/flywheel_constants.hpp"

using namespace tap::motor;
using namespace tap::algorithms;
using namespace src::flywheel;

namespace src::control::flywheel
{
FlywheelSubsystem::FlywheelSubsystem(
    tap::Drivers *drivers,
    tap::motor::REVMotorId leftMotorId,
    tap::motor::REVMotorId rightMotorId,
    tap::motor::REVMotorId upMotorId,
    tap::can::CanBus canBus)
    : tap::control::Subsystem(drivers),
      spinToRPMMap(SPIN_TO_INTERPOLATABLE_MPS_TO_RPM),
      feedforwardmap(SPIN_TO_INTERPOLATABLE_MPS_TO_DUTY),
      leftWheel(drivers, leftMotorId, canBus, false, "Left Flywheel"),
      rightWheel(drivers, rightMotorId, canBus, true, "Right Flywheel"),
      upWheel(drivers, upMotorId, canBus, true, "Up Flywheel"),
      desiredLaunchSpeedLeft(0),
      desiredLaunchSpeedRight(0),
      desiredLaunchSpeedUp(0),
      desiredRpmRampLeft(0),
      desiredRpmRampRight(0),
      desiredRpmRampUp(0),
      topFlyWheelPid(
          FLYWHEEL_DUTY_PID_KP, FLYWHEEL_DUTY_PID_KI, FLYWHEEL_DUTY_PID_KD,
          FLYWHEEL_DUTY_PID_MAX_ERROR_SUM, FLYWHEEL_DUTY_PID_MAX_OUTPUT),
        bottomLeftFlyWheelPid(
            FLYWHEEL_DUTY_PID_KP, FLYWHEEL_DUTY_PID_KI, FLYWHEEL_DUTY_PID_KD,
            FLYWHEEL_DUTY_PID_MAX_ERROR_SUM, FLYWHEEL_DUTY_PID_MAX_OUTPUT),
        bottomRightFlyWheelPid(
            FLYWHEEL_DUTY_PID_KP, FLYWHEEL_DUTY_PID_KI, FLYWHEEL_DUTY_PID_KD,
            FLYWHEEL_DUTY_PID_MAX_ERROR_SUM, FLYWHEEL_DUTY_PID_MAX_OUTPUT)
          {};

void FlywheelSubsystem::initialize()
{
    leftWheel.setControlMode(tap::motor::RevMotor::ControlMode::DUTY_CYCLE);
    rightWheel.setControlMode(tap::motor::RevMotor::ControlMode::DUTY_CYCLE);
    upWheel.setControlMode(tap::motor::RevMotor::ControlMode::DUTY_CYCLE);
    leftWheel.initialize();
    rightWheel.initialize();
    upWheel.initialize();
    // leftWheel.setControlMode(tap::motor::RevMotor::ControlMode::VELOCITY);
    // rightWheel.setControlMode(tap::motor::RevMotor::ControlMode::VELOCITY);
    // upWheel.setControlMode(tap::motor::RevMotor::ControlMode::VELOCITY);
}

void FlywheelSubsystem::setDesiredSpin(u_int16_t spin)
{
    if (auto spinSet = toSpinPreset(spin))
    {
        desiredSpin = spinSet.value();
        desiredSpinValue = spin;
    }
}

/**
 * using the set spin sets a desired rpm for the flywheels with the up wheel scaled by the spin
 * @param[in] speed in meters per second
 */
void FlywheelSubsystem::setDesiredLaunchSpeed(float speed)
{
    previousLaunchSpeedLeft = desiredLaunchSpeedLeft;
    previousLaunchSpeedRight = desiredLaunchSpeedRight;
    previousLaunchSpeedUp = desiredLaunchSpeedUp;


    desiredLaunchSpeedLeft = speed;
    desiredLaunchSpeedRight = speed;
    desiredLaunchSpeedUp = speed * (desiredSpinValue / 100.0f);

    previousTopSetPoint = desiredRpmRampUp.getTarget();
    previousLeftSetPoint = desiredRpmRampLeft.getTarget();
    previousRightSetPoint = desiredRpmRampRight.getTarget();

    desiredRpmRampLeft.setTarget(limitVal(launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft), 0.0f, MAX_DESIRED_LAUNCH_SPEED));
    desiredRpmRampRight.setTarget(limitVal(launchSpeedToFlywheelRpm(desiredLaunchSpeedRight), 0.0f, MAX_DESIRED_LAUNCH_SPEED));
    desiredRpmRampUp.setTarget(limitVal(launchSpeedToFlywheelRpm(desiredLaunchSpeedUp), 0.0f, MAX_DESIRED_LAUNCH_SPEED));
}
// void FlywheelSubsystem::setDesiredLaunchSpeedRight(float speed)
// {
//     desiredLaunchSpeedRight = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
//     desiredRpmRampRight.setTarget(launchSpeedToFlywheelRpm(speed));
// }
// void FlywheelSubsystem::setDesiredLaunchSpeedUp(float speed)
// {
//     desiredLaunchSpeedUp = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
//     desiredRpmRampUp.setTarget(launchSpeedToFlywheelRpm(speed));
// }

float FlywheelSubsystem::launchSpeedToFlywheelRpm(float launchSpeed) const
{
    modm::interpolation::Linear<modm::Pair<float, float>> MPSToRPMInterpolator = {
        spinToRPMMap.at(desiredSpin).data(),
        spinToRPMMap.at(desiredSpin).size()};
    return MPSToRPMInterpolator.interpolate(launchSpeed);
}
float upRpm = 0;
float rightRpm = 0;
float leftRpm = 0;
float upSetPoint = 0;
float leftSetPoint = 0;
float rightSetPoint = 0;
float topOutput = 0;
float leftOutput = 0;
float rightOutput = 0;
void FlywheelSubsystem::refresh()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    if (currTime == prevTime)
    {
        return;
    }

    desiredRpmRampLeft.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    desiredRpmRampRight.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    desiredRpmRampUp.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    prevTime = currTime;

    //invert pid output because the encoder is inverted
    topFlyWheelPid.update(desiredRpmRampUp.getValue() - -upWheel.getVelocity(), false);
    bottomLeftFlyWheelPid.update(desiredRpmRampLeft.getValue() - leftWheel.getVelocity(), false);
    bottomRightFlyWheelPid.update(desiredRpmRampRight.getValue() - -rightWheel.getVelocity(), false);

    modm::interpolation::Linear<modm::Pair<float, float>> MPSToDutyInterpolator = {
        feedforwardmap.at(desiredSpin).data(),
        feedforwardmap.at(desiredSpin).size()
    };

    float topRampingRatio = 1.0f;
    float leftRampingRatio = 1.0f;
    float rightRampingRatio = 1.0f;

    // Calculate delta and ratio for EACH wheel INDIVIDUALLY
    float rampDeltaTop = desiredRpmRampUp.getTarget() -  previousTopSetPoint;
    if (rampDeltaTop != 0.0f)
    {
       // Correct formula: (current - start) / (end - start)
       topRampingRatio = abs((desiredRpmRampUp.getValue() - previousTopSetPoint) / rampDeltaTop);
    }

    float rampDeltaLeft = desiredRpmRampLeft.getTarget() - previousLeftSetPoint;
    if (rampDeltaLeft != 0.0f)
    {
       leftRampingRatio = abs((desiredRpmRampLeft.getValue() - previousLeftSetPoint) / rampDeltaLeft);
    }


    //start 1000 target 0
    float rampDeltaRight = desiredRpmRampRight.getTarget() - previousRightSetPoint;
    if (rampDeltaRight != 0.0f)
    {
       rightRampingRatio = abs((desiredRpmRampRight.getValue() - previousRightSetPoint) / rampDeltaRight);
    }

    float start_ff_top = MPSToDutyInterpolator.interpolate(previousLaunchSpeedUp);
    float end_ff_top = MPSToDutyInterpolator.interpolate(desiredLaunchSpeedUp);

    float start_ff_left = MPSToDutyInterpolator.interpolate(previousLaunchSpeedLeft);
    float end_ff_left = MPSToDutyInterpolator.interpolate(desiredLaunchSpeedLeft);

    float start_ff_right = MPSToDutyInterpolator.interpolate(previousLaunchSpeedRight);
    float end_ff_right = MPSToDutyInterpolator.interpolate(desiredLaunchSpeedRight);

    // Linearly interpolate the feedforward value based on the ramp progress (rampingRatio)
    float topFlyWheelFeedforward = start_ff_top + (end_ff_top - start_ff_top) * topRampingRatio;
    float leftFlyWheelFeedforward = start_ff_left + (end_ff_left - start_ff_left) * leftRampingRatio;
    float rightFlyWheelFeedforward = start_ff_right + (end_ff_right - start_ff_right) * rightRampingRatio;



    float topFlywheelOutput = topFlyWheelPid.getValue() + topFlyWheelFeedforward;
    float bottomLeftFlywheelOutput = bottomLeftFlyWheelPid.getValue() + leftFlyWheelFeedforward;
    float bottomRightFlywheelOutput = bottomRightFlyWheelPid.getValue() + rightFlyWheelFeedforward;
    
    leftWheel.setControlValue(bottomLeftFlywheelOutput);
    rightWheel.setControlValue(bottomRightFlywheelOutput);
    upWheel.setControlValue(topFlywheelOutput);



    upRpm = upWheel.getVelocity();
    rightRpm = rightWheel.getVelocity();
    leftRpm = leftWheel.getVelocity();
    upSetPoint = desiredRpmRampUp.getValue();
    rightSetPoint = desiredRpmRampRight.getValue();
    leftSetPoint = desiredRpmRampLeft.getValue();
    topOutput = topFlywheelOutput;
    leftOutput = bottomLeftFlywheelOutput;
    rightOutput = bottomRightFlywheelOutput;
}
}  // namespace src::control::flywheel

#endif  // TARGET_HERO