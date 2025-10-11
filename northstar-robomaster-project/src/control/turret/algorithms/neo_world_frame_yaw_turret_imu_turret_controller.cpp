#include "neo_world_frame_yaw_turret_imu_turret_controller.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

#include "../turret_subsystem.hpp"
#include "control/turret/constants/turret_constants.hpp"

namespace src::control::turret::algorithms
{
/**
 * Transforms the specified `angleToTransform`, a yaw/pitch angle (in radians) from the chassis
 * frame to the world frame.
 *
 * @note It is expected that the user wraps the value returned to be between [0, M_TWOPI)
 *      (or whatever range they require).
 *
 * @param[in] turretChassisFrameCurrAngle The current chassis relative (gimbal encoder) angle.
 * @param[in] turretWorldFrameCurrAngle The current world relative (turret IMU) angle,
 *      captured at the same time as `turretChassisFrameCurrAngle`.
 * @param[in] angleToTransform The angle to transform.
 * @return The transformed angle in the world frame.
 */
static inline WrappedFloat transformChassisFrameToWorldFrame(
    const WrappedFloat turretChassisFrameCurrAngle,
    const WrappedFloat turretWorldFrameCurrAngle,
    const WrappedFloat angleToTransform)
{
    return turretWorldFrameCurrAngle + (angleToTransform - turretChassisFrameCurrAngle);
}

/**
 * Transforms the specified `angleToTransform`, a yaw or pitch angle (in radians), from the world
 * frame to the chassis frame.
 *
 * @note It is expected that the user wraps the value returned to be between [0, M_TWOPI)
 *      (or whatever range they require).
 *
 * @param[in] turretChassisFrameCurrAngle The current chassis relative (gimbal encoder) angle.
 * @param[in] turretWorldFrameCurrAngle The current world relative (turret IMU) angle, captured
 *      at the same time as `turretChassisFrameCurrAngle`.
 * @param[in] angleToTransform The angle to transform.
 * @return The transformed angle in the chassis frame.
 */
static inline WrappedFloat transformWorldFrameValueToChassisFrame(
    const WrappedFloat turretChassisFrameCurrAngle,  // motor
    const WrappedFloat turretWorldFrameCurrAngle,    // IMU
    const WrappedFloat angleToTransform)             // world frame setpoint
{
    return turretChassisFrameCurrAngle +
           (angleToTransform - turretWorldFrameCurrAngle);  // sets chassisFrameSetpoint
}

/**
 * Initializes a world frame cascade PID turret controller
 *
 * @param[in] controllerToInitialize The TurretControllerInterface in question being initialized.
 * @param[in] worldFrameMeasurement The measured world frame angle, in radians, not expected to be
 * normalized.
 * @param[out] turretMotor The turret motor that will be controlled by the passed in
 * controllerToInitialize.
 * @param[out] positionPid Position PID controller.
 * @param[out] velocityPid Velocity PID controller.
 * @param[out] worldFrameSetpoint World frame angle setpoint that will be set to the current
 * turretMotor's setpoint.
 */
static inline void initializeWorldFrameTurretImuController(
    const TurretControllerInterface *controllerToInitialize,
    const WrappedFloat worldFrameMeasurement,
    TurretMotor &turretMotor,
    tap::algorithms::SmoothPid &positionPid,
    tap::algorithms::SmoothPid &velocityPid,
    WrappedFloat &worldFrameSetpoint)
{
    if (turretMotor.getTurretController() != controllerToInitialize)
    {
        positionPid.reset();
        velocityPid.reset();

        worldFrameSetpoint = transformChassisFrameToWorldFrame(
            turretMotor.getChassisFrameMeasuredAngle(),
            worldFrameMeasurement,
            turretMotor.getChassisFrameSetpoint());

        turretMotor.attachTurretController(controllerToInitialize);
    }
}

/**
 * A helper function for the `run*PidYawWorldFrameController` functions below. Updates the passed in
 * `turretMotor`'s desired chassis frame setpoint and the passed in `worldFrameSetpoint`'.
 * Performs necessary limiting of the `worldFrameSetpoint` based on the `turretMotor`'s
 * min/max setpoints.
 *
 * @param[in] desiredSetpoint The new user-specified world frame turret motor angle setpoint, in
 * radians.
 * @param[in] chassisFrameMeasurement The chassis frame motor angle, in radians, measured by the
 * motor's encoder.
 * @param[in] worldFrameMeasurement The current chassis IMU angle, in radians, measured from the
 * chassis mounted IMU.
 * @param[out] worldFrameSetpoint The limited and wrapped world frame turret motor setpoint, in
 * radians. Set to `desiredSetpoint` and then wrapped/limited as necessary.
 * @param[out] turretMotor The turret subsystem whose chassis relative turret motor angle is
 * updated by this function.
 */
static inline void updateWorldFrameSetpoint(
    const WrappedFloat desiredSetpoint,
    const WrappedFloat chassisFrameMeasurement,  // motor
    const WrappedFloat worldFrameMeasurement,    // IMU
    WrappedFloat &worldFrameSetpoint,
    TurretMotor &turretMotor)
{
    worldFrameSetpoint = desiredSetpoint;

    // transform target angle from turret imu relative to chassis relative
    // to keep turret/command setpoints synchronized
    turretMotor.setChassisFrameSetpoint(transformWorldFrameValueToChassisFrame(
        chassisFrameMeasurement,
        worldFrameMeasurement,
        worldFrameSetpoint));

    if (turretMotor.getConfig().limitMotorAngles)
    {
        // transform angle that is limited by subsystem to world relative again to run the
        // controller
        worldFrameSetpoint = transformChassisFrameToWorldFrame(
            chassisFrameMeasurement,
            worldFrameMeasurement,
            turretMotor.getChassisFrameSetpoint());
    }
}

/**
 * Runs a world frame cascade (position -> velocity) PID controller.
 *
 * @param[in] worldFrameAngleSetpoint World frame angle setpoint, not required to be normalized, in
 * radians.
 * @param[in] worldFrameAngleMeasurement World frame angle measurement, not required to be
 * normalized, in radians.
 * @param[in] worldFrameVelocityMeasured World frame angular velocity measurement, in
 * radians/second.
 * @param[in] dt Time change since this function was last called, in ms.
 * @param[in] turretMotor TurretMotor associated with the angles being measured.
 * @param[out] positionPid Position PID controller.
 * @param[out] velocityPid Velocity PID controller.
 * @return desired PID output from running the position -> velocity cascade controller
 */

static inline float runWorldFrameTurretImuController(
    const WrappedFloat worldFrameAngleError,
    const WrappedFloat chassisFrameAngleMeasurement,
    const float worldFrameVelocityMeasured,
    const uint32_t dt,
    const TurretMotor &turretMotor,
    tap::algorithms::SmoothPid &positionPid,
    tap::algorithms::SmoothPid &velocityPid)
{
    const float positionControllerError = turretMotor.getValidMinError(
        chassisFrameAngleMeasurement + worldFrameAngleError,
        chassisFrameAngleMeasurement);
    const float positionPidOutput =
        positionPid.runController(positionControllerError, worldFrameVelocityMeasured, dt);

    const float velocityControllerError = positionPidOutput - worldFrameVelocityMeasured;
    const float velocityPidOutput =
        velocityPid.runControllerDerivateError(velocityControllerError, dt);
    return velocityPidOutput;
}

NeoWorldFrameYawTurretImuCascadePidTurretController::
    NeoWorldFrameYawTurretImuCascadePidTurretController(
        tap::Drivers &drivers,
        TurretMotor &yawMotor,
        tap::algorithms::SmoothPid &positionPid,
        tap::algorithms::SmoothPid &velocityPid)
    : TurretYawControllerInterface(yawMotor),
      drivers(drivers),
      positionPid(positionPid),
      velocityPid(velocityPid),
      worldFrameSetpoint(Angle(0)),
      worldFrameMeasurementIMU(0)
{
}

void NeoWorldFrameYawTurretImuCascadePidTurretController::initialize()
{
    initializeWorldFrameTurretImuController(
        this,
        getBmi088Yaw(),
        turretMotor,
        positionPid,
        velocityPid,
        worldFrameSetpoint);
}

void NeoWorldFrameYawTurretImuCascadePidTurretController::runController(
    const uint32_t dt,
    const WrappedFloat desiredSetpoint)
{
    if (abs(worldFrameMeasurementIMU - getBmi088Yaw(true).getWrappedValue()) > M_TWOPI * .7f)
    {
        IMUrevolutions += tap::algorithms::getSign(
            worldFrameMeasurementIMU - getBmi088Yaw(true).getWrappedValue());
    }
    worldFrameMeasurementIMU = getBmi088Yaw(true).getWrappedValue();
    const WrappedFloat chassisFrameYaw = turretMotor.getChassisFrameMeasuredAngle();
    const WrappedFloat worldFrameYawAngle = getBmi088Yaw(true);  // negitive
    const float worldFrameYawVelocity = -getBmi088YawVelocity();

    updateWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameYaw,
        worldFrameYawAngle,
        worldFrameSetpoint,
        turretMotor);

    const float pidOut = runWorldFrameTurretImuController(
        worldFrameSetpoint - worldFrameYawAngle,
        chassisFrameYaw,
        worldFrameYawVelocity,
        dt,
        turretMotor,
        positionPid,
        velocityPid);

    turretMotor.setMotorOutput(pidOut);
}

void NeoWorldFrameYawTurretImuCascadePidTurretController::setSetpoint(WrappedFloat desiredSetpoint)
{
    const WrappedFloat chassisFrameYaw = turretMotor.getChassisFrameMeasuredAngle();

    const WrappedFloat worldFrameYawAngle = getBmi088Yaw();

    updateWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameYaw,
        worldFrameYawAngle,
        worldFrameSetpoint,
        turretMotor);
}

WrappedFloat NeoWorldFrameYawTurretImuCascadePidTurretController::getMeasurement() const
{
    return Angle(worldFrameMeasurementIMU + M_TWOPI * IMUrevolutions);
}

WrappedFloat NeoWorldFrameYawTurretImuCascadePidTurretController::getMeasurementMotor() const
{
    return turretMotor.getChassisFrameMeasuredAngle();
}

bool NeoWorldFrameYawTurretImuCascadePidTurretController::isOnline() const
{
    return turretMotor.isOnline() &&
           (drivers.bmi088.getImuState() ==
                tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED ||
            drivers.bmi088.getImuState() ==
                tap::communication::sensors::imu::ImuInterface::ImuState::
                    IMU_NOT_CALIBRATED);  // TODO not shure if this is valid, was
                                          // drivers.mpu6500.isRunning();;
}

WrappedFloat NeoWorldFrameYawTurretImuCascadePidTurretController::
    convertControllerAngleToChassisFrame(WrappedFloat controllerFrameAngle) const
{
    const WrappedFloat worldFrameYawAngle = getBmi088Yaw();

    return transformWorldFrameValueToChassisFrame(
        turretMotor.getChassisFrameMeasuredAngle(),
        worldFrameYawAngle,
        controllerFrameAngle);
}

WrappedFloat NeoWorldFrameYawTurretImuCascadePidTurretController::
    convertChassisAngleToControllerFrame(WrappedFloat chassisFrameAngle) const
{
    const WrappedFloat worldFrameYawAngle = getBmi088Yaw();

    return transformChassisFrameToWorldFrame(
        turretMotor.getChassisFrameMeasuredAngle(),
        worldFrameYawAngle,
        chassisFrameAngle);
}

}  // namespace src::control::turret::algorithms
