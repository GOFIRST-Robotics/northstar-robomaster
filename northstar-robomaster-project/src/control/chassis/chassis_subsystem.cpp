#include "chassis_subsystem.hpp"

#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

using tap::algorithms::limitVal;

/*
    Chassis Subsystem uses a 2D coordinate system, using the ground as the XY plane
    +X: Right
    +Y: Forward
    +Rotation: CW
*/

namespace src::chassis
{
modm::Pair<int, float> lastComputedMaxWheelSpeed = CHASSIS_POWER_TO_MAX_SPEED_LUT[0];
modm::Pair<int, float> lastComputedMaxAccelSpeed = CHASSIS_POWER_TO_MAX_ACCEL_LUT[0];

ChassisSubsystem::ChassisSubsystem(
    tap::Drivers* drivers,
    const ChassisConfig& config,
    src::can::TurretMCBCanComm* turretMcbCanComm,
    tap::motor::DjiMotor* yawMotor,
    ChassisOdometry* chassisOdometry_)
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
      chassisOdometry(chassisOdometry_)
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
        getMaxWheelSpeed(drivers->refSerial.getRefSerialReceivingData(), getChassisPowerLimit());
    float allowedwheelSpeed = maxWheelSpeed - mpsToRpm(modm::Vector2f(vert, hor).getLength());
    // float allowedwheelSpeed =
    //     (maxWheelSpeed -
    //      ((abs(vert / MAX_CHASSIS_SPEED_MPS) + abs(hor / MAX_CHASSIS_SPEED_MPS)) *
    //      maxWheelSpeed));
    if (allowedwheelSpeed < 0.0f)
    {
        allowedwheelSpeed = 0.0f;
    }
    return (allowedwheelSpeed * (CHASSIS_GEAR_RATIO) * (M_TWOPI / 60.0f) * (WHEEL_DIAMETER_M / 2)) /
           DIST_TO_CENTER;
}

void ChassisSubsystem::setVelocityTurretDrive(float forward, float sideways, float rotational)
{
    float turretRot = getTurretYaw();
    driveBasedOnHeading(forward, sideways, rotational, turretRot);
}

void ChassisSubsystem::setVelocityFieldDrive(float forward, float sideways, float rotational)
{
    float robotHeading = fmod(drivers->bmi088.getYaw() + getTurretYaw(), 2 * M_PI);
    driveBasedOnHeading(forward, sideways, rotational, robotHeading);
}

float ChassisSubsystem::chassisSpeedRotationPID()  // make this take in the heading to follow then
                                                   // use with auto drive
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

    return lastComputedMaxWheelSpeed.second;
}

float ChassisSubsystem::getMaxAccelSpeed(bool refSerialOnline, float chassisPowerLimit)
{
    if (!refSerialOnline)
    {
        chassisPowerLimit = 80;
    }

    // only re-interpolate when needed (since this function is called a lot and the chassis
    // power limit rarely changes, this helps cut down on unnecessary array
    // searching/interpolation)
    if (lastComputedMaxAccelSpeed.first != (int)chassisPowerLimit)
    {
        lastComputedMaxAccelSpeed.first = (int)chassisPowerLimit;
        lastComputedMaxAccelSpeed.second =
            CHASSIS_POWER_TO_ACCEL_INTERPOLATOR.interpolate(chassisPowerLimit);
    }

    return lastComputedMaxAccelSpeed.second;
}

void ChassisSubsystem::driveBasedOnHeading(
    float forward,
    float sideways,
    float rotational,
    float heading)
{
    float maxAccelSpeed =
        getMaxAccelSpeed(drivers->refSerial.getRefSerialReceivingData(), getChassisPowerLimit());
    rampControllers[0].setTarget(forward);
    rampControllers[0].update(
        abs(forward) < abs(rampControllers[0].getValue()) ? CHASSIS_DECCEL_VALUE : maxAccelSpeed);
    float rampedForward = rampControllers[0].getValue();
    rampControllers[1].setTarget(sideways);
    rampControllers[1].update(
        abs(sideways) < abs(rampControllers[1].getValue()) ? CHASSIS_DECCEL_VALUE : maxAccelSpeed);
    float rampedSideways = rampControllers[1].getValue();
    double cos_theta = cos(heading);
    double sin_theta = sin(heading);
    double vx_local = rampedSideways * cos_theta + rampedForward * sin_theta;
    double vy_local = -rampedSideways * sin_theta + rampedForward * cos_theta;
    LFSpeed = mpsToRpm(
        (vy_local + vx_local) / M_SQRT2 +
        (rotational)*DIST_TO_CENTER * M_SQRT2);  // Front-left wheel
    RFSpeed = mpsToRpm(
        (-vy_local + vx_local) / M_SQRT2 +
        (rotational)*DIST_TO_CENTER * M_SQRT2);  // Front-right wheel
    RBSpeed = mpsToRpm(
        (-vy_local - vx_local) / M_SQRT2 +
        (rotational)*DIST_TO_CENTER * M_SQRT2);  // Rear-right wheel
    LBSpeed = mpsToRpm(
        (vy_local - vx_local) / M_SQRT2 +
        (rotational)*DIST_TO_CENTER * M_SQRT2);  // Rear-left wheel
    int LF = static_cast<int>(MotorId::LF);
    int LB = static_cast<int>(MotorId::LB);
    int RF = static_cast<int>(MotorId::RF);
    int RB = static_cast<int>(MotorId::RB);
    float calculatedMaxRPMPower = limitVal<float>(
        getMaxWheelSpeed(drivers->refSerial.getRefSerialReceivingData(), getChassisPowerLimit()),
        -MAX_CHASSIS_WHEEL_SPEED,
        MAX_CHASSIS_WHEEL_SPEED);
    desiredOutput[LF] = limitVal<float>(LFSpeed, -calculatedMaxRPMPower, calculatedMaxRPMPower);
    desiredOutput[LB] = limitVal<float>(LBSpeed, -calculatedMaxRPMPower, calculatedMaxRPMPower);
    desiredOutput[RF] = limitVal<float>(RFSpeed, -calculatedMaxRPMPower, calculatedMaxRPMPower);
    desiredOutput[RB] = limitVal<float>(RBSpeed, -calculatedMaxRPMPower, calculatedMaxRPMPower);
}

float xPosOdometry;
float yPosOdometry;
float xVelOdometry;
float yVelOdometry;
float xVelGlobOdometry;
float yVelGlobOdometry;
float rotationOdometry;
float rotationOdometryDegrees;
float xl;

void ChassisSubsystem::refresh()
{
    auto runPid = [](Pid& pid, Motor& motor, float desiredOutput) {
        pid.update(
            desiredOutput -
            motor.getEncoder()->getVelocity() * 60.0f / M_TWOPI / CHASSIS_GEAR_RATIO);
        motor.setDesiredOutput(pid.getValue());
    };

    for (size_t ii = 0; ii < motors.size(); ii++)
    {
        runPid(pidControllers[ii], motors[ii], desiredOutput[ii]);
    }

    chassisOdometry->updateOdometry(
        motors[static_cast<int>(MotorId::LF)].getEncoder()->getVelocity(),
        motors[static_cast<int>(MotorId::LB)].getEncoder()->getVelocity(),
        motors[static_cast<int>(MotorId::RF)].getEncoder()->getVelocity(),
        motors[static_cast<int>(MotorId::RB)].getEncoder()->getVelocity());

    xPosOdometry = chassisOdometry->getPositionGlobal().x;
    yPosOdometry = chassisOdometry->getPositionGlobal().y;
    xVelOdometry = chassisOdometry->getVelocityGlobal().x;
    yVelOdometry = chassisOdometry->getVelocityGlobal().y;
    xl = chassisOdometry->getVelocityLocal().x;
    xVelGlobOdometry = chassisOdometry->getVelocityProjectedGlobal().x;
    yVelGlobOdometry = chassisOdometry->getVelocityProjectedGlobal().y;
    rotationOdometry = chassisOdometry->getRotation();
    rotationOdometryDegrees = modm::toDegree(rotationOdometry);
}
}  // namespace src::chassis
