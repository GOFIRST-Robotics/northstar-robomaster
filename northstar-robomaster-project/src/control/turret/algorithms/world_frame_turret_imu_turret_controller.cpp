/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "world_frame_turret_imu_turret_controller.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

#include "../turret_subsystem.hpp"
#include "control/turret/constants/turret_constants.hpp"

#include "turret_gravity_compensation.hpp"

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
float debugpositionControllerError;
float debugvelocityPidOutput;
float debugworldFrameAngleError;
float debugchassisFrameAngleMeasurement;

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
    debugchassisFrameAngleMeasurement = chassisFrameAngleMeasurement.getUnwrappedValue();
    debugworldFrameAngleError = worldFrameAngleError.getUnwrappedValue();
    debugpositionControllerError = positionControllerError;
    const float positionPidOutput =
        positionPid.runController(positionControllerError, worldFrameVelocityMeasured, dt);

    const float velocityControllerError = positionPidOutput - worldFrameVelocityMeasured;
    const float velocityPidOutput =
        velocityPid.runControllerDerivateError(velocityControllerError, dt);
    debugvelocityPidOutput = velocityPidOutput;
    return velocityPidOutput;
}

WorldFrameYawTurretImuCascadePidTurretController::WorldFrameYawTurretImuCascadePidTurretController(
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

void WorldFrameYawTurretImuCascadePidTurretController::initialize()
{
    initializeWorldFrameTurretImuController(
        this,
        getBmi088Yaw(),
        turretMotor,
        positionPid,
        velocityPid,
        worldFrameSetpoint);
}

void WorldFrameYawTurretImuCascadePidTurretController::runController(
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

void WorldFrameYawTurretImuCascadePidTurretController::setSetpoint(WrappedFloat desiredSetpoint)
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

WrappedFloat WorldFrameYawTurretImuCascadePidTurretController::getMeasurement() const
{
    return Angle(worldFrameMeasurementIMU + M_TWOPI * IMUrevolutions);
}

WrappedFloat WorldFrameYawTurretImuCascadePidTurretController::getMeasurementMotor() const
{
    return turretMotor.getChassisFrameMeasuredAngle();
}

bool WorldFrameYawTurretImuCascadePidTurretController::isOnline() const
{
    return turretMotor.isOnline() &&
           (drivers.bmi088.getImuState() ==
                tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED ||
            drivers.bmi088.getImuState() ==
                tap::communication::sensors::imu::ImuInterface::ImuState::
                    IMU_NOT_CALIBRATED);  // TODO not shure if this is valid, was
                                          // drivers.mpu6500.isRunning();;
}

WrappedFloat WorldFrameYawTurretImuCascadePidTurretController::convertControllerAngleToChassisFrame(
    WrappedFloat controllerFrameAngle) const
{
    const WrappedFloat worldFrameYawAngle = getBmi088Yaw();

    return transformWorldFrameValueToChassisFrame(
        turretMotor.getChassisFrameMeasuredAngle(),
        worldFrameYawAngle,
        controllerFrameAngle);
}

WrappedFloat WorldFrameYawTurretImuCascadePidTurretController::convertChassisAngleToControllerFrame(
    WrappedFloat chassisFrameAngle) const
{
    const WrappedFloat worldFrameYawAngle = getBmi088Yaw();

    return transformChassisFrameToWorldFrame(
        turretMotor.getChassisFrameMeasuredAngle(),
        worldFrameYawAngle,
        chassisFrameAngle);
}

WorldFramePitchTurretImuCascadePidTurretController::
    WorldFramePitchTurretImuCascadePidTurretController(
        tap::Drivers &drivers,
        TurretMotor &turretMotor,
        tap::algorithms::SmoothPid &positionPid,
        tap::algorithms::SmoothPid &velocityPid)
    : TurretPitchControllerInterface(turretMotor),
      drivers(drivers),
      positionPid(positionPid),
      velocityPid(velocityPid),
      worldFrameSetpoint(Angle(0))
{
}

void WorldFramePitchTurretImuCascadePidTurretController::initialize()
{
    initializeWorldFrameTurretImuController(
        this,
        getBmi088Pitch(),
        turretMotor,
        positionPid,
        velocityPid,
        worldFrameSetpoint);
}
float debugpidOut = 0;
void WorldFramePitchTurretImuCascadePidTurretController::runController(  // TODO for actual use
                                                                         // change back to pitch
    const uint32_t dt,
    const WrappedFloat desiredSetpoint)
{
    const WrappedFloat chassisFramePitch = turretMotor.getChassisFrameMeasuredAngle();
    const WrappedFloat worldFramePitchAngle = getBmi088Pitch(false);  // negitive
    const float worldFramePitchVelocity = getBmi088PitchVelocity();

    updateWorldFrameSetpoint(
        desiredSetpoint,
        chassisFramePitch,
        worldFramePitchAngle,
        worldFrameSetpoint,
        turretMotor);

    float pidOut = runWorldFrameTurretImuController(
        worldFrameSetpoint - worldFramePitchAngle,
        chassisFramePitch,
        worldFramePitchVelocity,
        dt,
        turretMotor,
        positionPid,
        velocityPid);
    debugpidOut = pidOut;
    // pidOut += computeGravitationalForceOffset(
    //     TURRET_CG_X,
    //     TURRET_CG_Z,
    //     turretMotor.getChassisFrameMeasuredAngle().getWrappedValue() - M_PI / 2,
    //     GRAVITY_COMPENSATION_SCALAR);
    debugpidOut = pidOut;
    turretMotor.setMotorOutput(pidOut);
}

void WorldFramePitchTurretImuCascadePidTurretController::setSetpoint(WrappedFloat desiredSetpoint)
{
    const WrappedFloat chassisFramePitch = turretMotor.getChassisFrameMeasuredAngle();
    const WrappedFloat worldFramePitchAngle = getBmi088Pitch();

    updateWorldFrameSetpoint(
        desiredSetpoint,
        chassisFramePitch,
        worldFramePitchAngle,
        worldFrameSetpoint,
        turretMotor);
}

WrappedFloat WorldFramePitchTurretImuCascadePidTurretController::getMeasurement() const
{
    return getBmi088Pitch();
}

bool WorldFramePitchTurretImuCascadePidTurretController::isOnline() const
{
    return turretMotor.isOnline() &&
           (drivers.bmi088.getImuState() ==
                tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED ||
            drivers.bmi088.getImuState() ==
                tap::communication::sensors::imu::ImuInterface::ImuState::
                    IMU_NOT_CALIBRATED);  // TODO not shure if this is valid, was
                                          // drivers.mpu6500.isRunning();;
}

WrappedFloat WorldFramePitchTurretImuCascadePidTurretController::
    convertControllerAngleToChassisFrame(WrappedFloat controllerFrameAngle) const
{
    const WrappedFloat worldFramePitchAngle = getBmi088Pitch();

    return transformWorldFrameValueToChassisFrame(
        turretMotor.getChassisFrameMeasuredAngle(),
        worldFramePitchAngle,
        controllerFrameAngle);
}

WrappedFloat WorldFramePitchTurretImuCascadePidTurretController::
    convertChassisAngleToControllerFrame(WrappedFloat chassisFrameAngle) const
{
    const WrappedFloat worldFramePitchAngle = getBmi088Pitch();

    return transformChassisFrameToWorldFrame(
        turretMotor.getChassisFrameMeasuredAngle(),
        worldFramePitchAngle,
        chassisFrameAngle);
}
}  // namespace src::control::turret::algorithms
