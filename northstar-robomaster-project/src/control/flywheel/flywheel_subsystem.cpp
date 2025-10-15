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
    tap::can::CanBus canBus,
    RevMotor::PIDConfig pidConfig)
    : tap::control::Subsystem(drivers),
      spinToRPMMap(SPIN_TO_INTERPOLATABLE_MPS_TO_RPM),
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
      upWheel(drivers, upMotorId, canBus, RevMotor::ControlMode::VELOCITY, true, "Up Flywheel"),
      desiredLaunchSpeedLeft(0),
      desiredLaunchSpeedRight(0),
      desiredLaunchSpeedUp(0),
      desiredRpmRampLeft(0),
      desiredRpmRampRight(0),
      desiredRpmRampUp(0),
      pidConfig(pidConfig)
{
}

void FlywheelSubsystem::initialize()
{
    leftWheel.initialize();
    rightWheel.initialize();
    upWheel.initialize();

    leftWheel.setMotorPID(pidConfig);
    rightWheel.setMotorPID(pidConfig);
    upWheel.setMotorPID(pidConfig);
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

float debugLeft = 0;
float debugRight = 0;
float debugUp = 0;
float debugLeftD = 0;
float debugRightD = 0;
float debugUpD = 0;

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

    debugLeft = leftWheel.getEncoder()->getVelocity() * 60 / (2 * M_PI) * 60;
    debugRight = rightWheel.getEncoder()->getVelocity() * 60 / (2 * M_PI) * 60;
    debugUp = upWheel.getEncoder()->getVelocity() * 60 / (2 * M_PI) * 60;

    debugLeftD = desiredRpmRampLeft.getValue();
    debugRightD = desiredRpmRampRight.getValue();
    debugUpD = desiredRpmRampUp.getValue();
}
}  // namespace src::control::flywheel

#endif  // TARGET_HERO