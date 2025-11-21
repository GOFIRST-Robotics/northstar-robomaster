#include "rev_three_flywheel_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "control/flywheel/flywheel_constants.hpp"

using namespace tap::motor;
using namespace tap::algorithms;
using namespace src::flywheel;

namespace src::control::flywheel
{
RevThreeFlywheelSubsystem::RevThreeFlywheelSubsystem(
    tap::Drivers *drivers,
    tap::motor::REVMotorId leftMotorId,
    tap::motor::REVMotorId rightMotorId,
    tap::motor::REVMotorId upMotorId,
    tap::can::CanBus canBus)
    : ThreeFlywheelSubsystem(drivers, SPIN_TO_INTERPOLATABLE_MPS_TO_RPM),
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

void RevThreeFlywheelSubsystem::initialize()
{
    leftWheel.initialize();
    rightWheel.initialize();
    upWheel.initialize();

    leftWheel.setMotorPID(pidConfig);
    rightWheel.setMotorPID(pidConfig);
    upWheel.setMotorPID(pidConfig);
}

void RevThreeFlywheelSubsystem::setDesiredSpin(u_int16_t spin)
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
void RevThreeFlywheelSubsystem::setDesiredLaunchSpeed(float speed)
{
    desiredLaunchSpeedLeft = speed;
    desiredLaunchSpeedRight = speed;
    desiredLaunchSpeedUp = speed * (desiredSpinValue / 100.0f);

    desiredRpmRampLeft.setTarget(limitVal(
        launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft),
        0.0f,
        MAX_DESIRED_LAUNCH_SPEED_RPM));

    desiredRpmRampRight.setTarget(limitVal(
        launchSpeedToFlywheelRpm(desiredLaunchSpeedRight),
        0.0f,
        MAX_DESIRED_LAUNCH_SPEED_RPM));

    desiredRpmRampUp.setTarget(limitVal(
        launchSpeedToFlywheelRpm(desiredLaunchSpeedUp),
        0.0f,
        MAX_DESIRED_LAUNCH_SPEED_RPM));
}

void RevThreeFlywheelSubsystem::setDesiredFlywheelSpeed(float rpm)
{
    desiredRpmRampLeft.setTarget(launchSpeedToFlywheelRpm(rpm));
    desiredRpmRampRight.setTarget(launchSpeedToFlywheelRpm(rpm));
    desiredRpmRampUp.setTarget(launchSpeedToFlywheelRpm(rpm * (desiredSpinValue / 100.0f)));
}

float RevThreeFlywheelSubsystem::launchSpeedToFlywheelRpm(float launchSpeed) const
{
    modm::interpolation::Linear<modm::Pair<float, float>> MPSToRPMInterpolator = {
        spinToRPMMap.at(desiredSpin).data(),
        spinToRPMMap.at(desiredSpin).size()};
    return MPSToRPMInterpolator.interpolate(launchSpeed);
}

float debugLeft2 = 0;
float debugRight2 = 0;
float debugUp2 = 0;
float debugLeftD2 = 0;
float debugRightD2 = 0;
float debugUpD2 = 0;

void RevThreeFlywheelSubsystem::refresh()
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

    debugLeft2 = leftWheel.getEncoder()->getVelocity() * 60 / (2 * M_PI);
    debugRight2 = rightWheel.getEncoder()->getVelocity() * 60 / (2 * M_PI);
    debugUp2 = upWheel.getEncoder()->getVelocity() * 60 / (2 * M_PI);

    debugLeftD2 = desiredRpmRampLeft.getValue();
    debugRightD2 = desiredRpmRampRight.getValue();
    debugUpD2 = desiredRpmRampUp.getValue();
}
}  // namespace src::control::flywheel