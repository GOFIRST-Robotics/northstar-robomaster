#include "chassis_subsystem.hpp"

#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
modm::Pair<int, float> lastComputedMaxWheelSpeed = CHASSIS_POWER_TO_MAX_SPEED_LUT[0];
modm::Pair<int, float> lastComputedMaxAccelSpeed = CHASSIS_POWER_TO_MAX_ACCEL_LUT[0];

ChassisSubsystem::ChassisSubsystem(
    tap::Drivers* drivers,
    const ChassisConfig& config,
    src::can::TurretMCBCanComm* turretMcbCanComm,
    tap::motor::DjiMotor* yawMotor,
    src::can::capbank::CapacitorBank* superCap)
    : Subsystem(drivers),
      desiredOutput{},
      pidControllers{
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT)},
      motors{
          Motor(drivers, config.leftFrontId, config.canBus, false, "LF", false, CHASSIS_GEAR_RATIO),
          Motor(drivers, config.leftBackId, config.canBus, false, "LB", false, CHASSIS_GEAR_RATIO),
          Motor(
              drivers,
              config.rightFrontId,
              config.canBus,
              false,
              "RF",
              false,
              CHASSIS_GEAR_RATIO),
          Motor(drivers, config.rightBackId, config.canBus, false, "RB", false, CHASSIS_GEAR_RATIO),
      },
      turretMcbCanComm(turretMcbCanComm),
      yawMotor(yawMotor),
      superCap(superCap)
{
}

void ChassisSubsystem::initialize()
{
    for (auto& i : motors)
    {
        i.initialize();
    }
}
float LFSpeed;
float LBSpeed;
float RFSpeed;
float RBSpeed;

float topheading;
float bottomheading;
float difference;

inline float ChassisSubsystem::getTurretYaw() { return yawMotor->getPositionWrapped(); }

float ChassisSubsystem::getChassisZeroTurret()
{
    float angle = (getTurretYaw());
    return (angle > M_PI) ? angle - M_TWOPI : angle;
}

float ChassisSubsystem::getChassisRotationSpeed()
{
    float motorSum = 0.0f;
    for (const Motor& i : motors)
    {
        motorSum += i.getEncoder()->getVelocity();
    }
    return (WHEEL_DIAMETER_M / (2 * DIST_TO_CENTER)) * motorSum;
}

float ChassisSubsystem::calculateMaxRotationSpeed(float vert, float hor)
{
    float maxWheelSpeed =
        getMaxWheelSpeed(drivers->refSerial.getRefSerialReceivingData(), getChassiPowerLimit());
    float allowedwheelSpeed =
        (maxWheelSpeed -
         ((abs(vert / MAX_CHASSIS_SPEED_MPS) + abs(hor / MAX_CHASSIS_SPEED_MPS)) * maxWheelSpeed));
    if (allowedwheelSpeed < 0.0f)
    {
        allowedwheelSpeed = 0.0f;
    }
    return (allowedwheelSpeed * (CHASSIS_GEAR_RATIO) * (M_TWOPI / 60.0f) * (WHEEL_DIAMETER_M / 2)) /
           DIST_TO_CENTER;
}

void ChassisSubsystem::setVelocityTurretDrive(float forward, float sideways, float rotational)
{
    // float turretRot = -getTurretYaw() + drivers->bmi088.getYaw();
    float turretRot = getTurretYaw();
    if (turretRot > M_TWOPI)
    {
        turretRot -= M_TWOPI;
    }
    else if (turretRot < 0.0f)
    {
        turretRot += M_TWOPI;
    }
    driveBasedOnHeading(forward, sideways, rotational, turretRot);
}

void ChassisSubsystem::setVelocityFieldDrive(float forward, float sideways, float rotational)
{
    float robotHeading = fmod(drivers->bmi088.getYaw() + getTurretYaw(), 2 * M_PI);
    driveBasedOnHeading(forward, sideways, rotational, robotHeading);
}

float ChassisSubsystem::chassisSpeedRotationPID()
{
    // P
    float currRotationPidP = getChassisZeroTurret() * CHASSIS_ROTATION_P;  // P
    currRotationPidP =
        limitVal<float>(currRotationPidP, -CHASSIS_ROTATION_MAX_VEL, CHASSIS_ROTATION_MAX_VEL);

    // D
    float currentRotationPidD = -(drivers->bmi088.getGz()) * CHASSIS_ROTATION_D;  // D

    currentRotationPidD = limitVal<float>(currentRotationPidD, -1, 1);

    float chassisRotationSpeed = limitVal<float>(currRotationPidP + currentRotationPidD, -1, 1);

    return chassisRotationSpeed;
}

float ChassisSubsystem::getMaxWheelSpeed(bool refSerialOnline, float chassisPowerLimit)
{
    if (!refSerialOnline)
    {
        chassisPowerLimit = 80;
    }

    // only re-interpolate when needed (since this function is called a lot and the chassis
    // power limit rarely changes, this helps cut down on unnecessary array
    // searching/interpolation)
    if (lastComputedMaxWheelSpeed.first != (int)chassisPowerLimit)
    {
        lastComputedMaxWheelSpeed.first = (int)chassisPowerLimit;
        lastComputedMaxWheelSpeed.second =
            CHASSIS_POWER_TO_SPEED_INTERPOLATOR.interpolate(chassisPowerLimit);
    }

    if (isSprinting && superCap->canSprint())
    {
        lastComputedMaxWheelSpeed.second = CHASSIS_POWER_TO_SPEED_INTERPOLATOR.interpolate(
            chassisPowerLimit + superCap->getAllowedSprintWattage());
    }

    return lastComputedMaxWheelSpeed.second;
}

void ChassisSubsystem::driveBasedOnHeading(
    float forward,
    float sideways,
    float rotational,
    float heading)
{
    double cos_theta = cos(heading);
    double sin_theta = sin(heading);
    double vx_local = forward * cos_theta + sideways * sin_theta;
    double vy_local = -forward * sin_theta + sideways * cos_theta;
    double sqrt2 = sqrt(2.0);
    LFSpeed = mpsToRpm(
        (vx_local - vy_local) / sqrt2 + (rotational)*DIST_TO_CENTER * sqrt2);  // Front-left wheel
    RFSpeed = mpsToRpm(
        (-vx_local - vy_local) / sqrt2 + (rotational)*DIST_TO_CENTER * sqrt2);  // Front-right wheel
    RBSpeed = mpsToRpm(
        (-vx_local + vy_local) / sqrt2 + (rotational)*DIST_TO_CENTER * sqrt2);  // Rear-right wheel
    LBSpeed = mpsToRpm(
        (vx_local + vy_local) / sqrt2 + (rotational)*DIST_TO_CENTER * sqrt2);  // Rear-left wheel
    int LF = static_cast<int>(MotorId::LF);
    int LB = static_cast<int>(MotorId::LB);
    int RF = static_cast<int>(MotorId::RF);
    int RB = static_cast<int>(MotorId::RB);
    float calculatedMaxRPMPower =
        getMaxWheelSpeed(drivers->refSerial.getRefSerialReceivingData(), getChassiPowerLimit());
    desiredOutput[LF] = limitVal<float>(LFSpeed, -calculatedMaxRPMPower, calculatedMaxRPMPower);
    desiredOutput[LB] = limitVal<float>(LBSpeed, -calculatedMaxRPMPower, calculatedMaxRPMPower);
    desiredOutput[RF] = limitVal<float>(RFSpeed, -calculatedMaxRPMPower, calculatedMaxRPMPower);
    desiredOutput[RB] = limitVal<float>(RBSpeed, -calculatedMaxRPMPower, calculatedMaxRPMPower);
}

void ChassisSubsystem::refresh()
{
    auto runPid = [](Pid& pid,
                     tap::algorithms::Ramp& ramp,
                     Motor& motor,
                     float desiredOutput,
                     float increment) {
        ramp.setTarget(desiredOutput);
        ramp.update(increment);
        pid.update(
            ramp.getValue() -
            motor.getEncoder()->getVelocity() * 60.0f / M_TWOPI / CHASSIS_GEAR_RATIO);
        motor.setDesiredOutput(pid.getValue());
    };

    for (size_t ii = 0; ii < motors.size(); ii++)
    {
        float calculatedMaxRPMAccel =
            getMaxAccelSpeed(drivers->refSerial.getRefSerialReceivingData(), getChassiPowerLimit());
        if (abs(motors[ii].getEncoder()->getVelocity() * 60.0f / M_TWOPI / CHASSIS_GEAR_RATIO) <
            abs(desiredOutput[ii]))
        {
            runPid(
                pidControllers[ii],
                rampControllers[ii],
                motors[ii],
                desiredOutput[ii],
                mpsToRpm(getMaxAccelSpeed(
                    drivers->refSerial.getRefSerialReceivingData(),
                    getChassiPowerLimit())));
        }
        else
        {
            runPid(
                pidControllers[ii],
                rampControllers[ii],
                motors[ii],
                desiredOutput[ii],
                mpsToRpm(CHASSIS_DECCEL_VALUE));
        }
    }
}
}  // namespace src::chassis
