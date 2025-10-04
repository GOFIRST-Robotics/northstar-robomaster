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
      leftWheel(drivers, leftMotorId, canBus, false, "Left Flywheel"),
      rightWheel(drivers, rightMotorId, canBus, true, "Right Flywheel"),
      upWheel(drivers, upMotorId, canBus, true, "Up Flywheel"),
      desiredLaunchSpeedLeft(0),
      desiredLaunchSpeedRight(0),
      desiredLaunchSpeedUp(0),
      desiredRpmRampLeft(0),
      desiredRpmRampRight(0),
      desiredRpmRampUp(0)
{
}

void FlywheelSubsystem::initialize()
{
    leftWheel.initialize();
    rightWheel.initialize();
    upWheel.initialize();

    leftWheel.setParameter(Parameter::kP_0, FLYWHEEL_PID_KP);
    leftWheel.setParameter(Parameter::kI_0, FLYWHEEL_PID_KI);
    leftWheel.setParameter(Parameter::kD_0, FLYWHEEL_PID_KD);
    leftWheel.setParameter(Parameter::kF_0, FLYWHEEL_PID_KF);
    leftWheel.setParameter(Parameter::kOutputMax_0, FLYWHEEL_PID_K_MAX_OUT);
    leftWheel.setParameter(Parameter::kOutputMin_0, FLYWHEEL_PID_K_MIN_OUT);

    rightWheel.setParameter(Parameter::kP_0, FLYWHEEL_PID_KP);
    rightWheel.setParameter(Parameter::kI_0, FLYWHEEL_PID_KI);
    rightWheel.setParameter(Parameter::kD_0, FLYWHEEL_PID_KD);
    rightWheel.setParameter(Parameter::kF_0, FLYWHEEL_PID_KF);
    rightWheel.setParameter(Parameter::kOutputMax_0, FLYWHEEL_PID_K_MAX_OUT);
    rightWheel.setParameter(Parameter::kOutputMin_0, FLYWHEEL_PID_K_MIN_OUT);

    upWheel.setParameter(Parameter::kP_0, FLYWHEEL_PID_KP);
    upWheel.setParameter(Parameter::kI_0, FLYWHEEL_PID_KI);
    upWheel.setParameter(Parameter::kD_0, FLYWHEEL_PID_KD);
    upWheel.setParameter(Parameter::kF_0, FLYWHEEL_PID_KF);
    upWheel.setParameter(Parameter::kOutputMax_0, FLYWHEEL_PID_K_MAX_OUT);
    upWheel.setParameter(Parameter::kOutputMin_0, FLYWHEEL_PID_K_MIN_OUT);

    leftWheel.setControlMode(tap::motor::RevMotor::ControlMode::VELOCITY);
    rightWheel.setControlMode(tap::motor::RevMotor::ControlMode::VELOCITY);
    upWheel.setControlMode(tap::motor::RevMotor::ControlMode::VELOCITY);
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
    desiredLaunchSpeedLeft = speed;
    desiredLaunchSpeedRight = speed;
    desiredLaunchSpeedUp = speed * (desiredSpinValue / 100.0f);

    desiredRpmRampLeft.setTarget(
        limitVal(launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft), 0.0f, MAX_DESIRED_LAUNCH_SPEED));

    desiredRpmRampRight.setTarget(limitVal(
        launchSpeedToFlywheelRpm(desiredLaunchSpeedRight),
        0.0f,
        MAX_DESIRED_LAUNCH_SPEED));

    desiredRpmRampUp.setTarget(
        limitVal(launchSpeedToFlywheelRpm(desiredLaunchSpeedUp), 0.0f, MAX_DESIRED_LAUNCH_SPEED));
}

float FlywheelSubsystem::launchSpeedToFlywheelRpm(float launchSpeed) const
{
    modm::interpolation::Linear<modm::Pair<float, float>> MPSToRPMInterpolator = {
        spinToRPMMap.at(desiredSpin).data(),
        spinToRPMMap.at(desiredSpin).size()};
    return MPSToRPMInterpolator.interpolate(launchSpeed);
}

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

    leftWheel.setControlValue(desiredRpmRampLeft.getValue());
    rightWheel.setControlValue(desiredRpmRampRight.getValue());
    upWheel.setControlValue(desiredRpmRampUp.getValue());
}
}  // namespace src::control::flywheel

#endif  // TARGET_HERO