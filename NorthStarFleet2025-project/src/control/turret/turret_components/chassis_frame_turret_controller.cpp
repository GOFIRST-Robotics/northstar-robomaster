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

#include "chassis_frame_turret_controller.hpp"

#include "tap/drivers.hpp"

#include "../turret_constants/standard_turret_constants.hpp"
#include "../turret_super_structure/turret_subsystem.hpp"

#include "../algorithms/turret_gravity_compensation.hpp"

using namespace tap::control::turret;

namespace control::turret::algorithms
{
ChassisFrameYawTurretController::ChassisFrameYawTurretController(
    TurretMotor &yawMotor,
    const tap::algorithms::SmoothPidConfig &pidConfig)
    : TurretYawControllerInterface(yawMotor),
      pid(pidConfig)
{
}

void ChassisFrameYawTurretController::initialize()
{
    //over ridden method
    turretMotor.initialize();
    if (turretMotor.getTurretController() != this)
    {
        pid.reset();
        turretMotor.attachTurretController(this);
    }
}


// float unwrappedPrimaryYawDebug = 0;
// float positionControllerErrorDebug = 0;
// float desiredYawSetpointDebug = 0;
// float debugPidOutput = 0;
void ChassisFrameYawTurretController::runController(const uint32_t dt, const float desiredSetpoint)
{
    // limit the yaw min and max angles
    turretMotor.updateMotorAngle();
    turretMotor.setChassisFrameSetpoint(desiredSetpoint);

    // desiredYawSetpointDebug = turretMotor.getChassisFrameSetpoint();

    // position controller based on turret yaw gimbal
    float unwrappedPrimaryYaw = turretMotor.getChassisFrameUnwrappedMeasuredAngle();
    // unwrappedPrimaryYawDebug = unwrappedPrimaryYaw;
    float positionControllerError = turretMotor.getValidMinError(turretMotor.getChassisFrameSetpoint(), unwrappedPrimaryYaw);

    

    float pidOutput =
        pid.runController(positionControllerError, turretMotor.getChassisFrameVelocity(), dt);

    // debugPidOutput = pidOutput;

    turretMotor.setMotorOutput(pidOutput);
}

void ChassisFrameYawTurretController::setSetpoint(float desiredSetpoint)
{
    turretMotor.setChassisFrameSetpoint(desiredSetpoint);
}

float ChassisFrameYawTurretController::getSetpoint() const
{
    return turretMotor.getChassisFrameSetpoint();
}

float ChassisFrameYawTurretController::getMeasurement() const
{
    return turretMotor.getChassisFrameUnwrappedMeasuredAngle();
}

bool ChassisFrameYawTurretController::isOnline() const { return turretMotor.isOnline(); }







ChassisFramePitchTurretController::ChassisFramePitchTurretController(
    TurretMotor &pitchMotorp,
    const tap::algorithms::SmoothPidConfig &pidConfig)
    : TurretPitchControllerInterface(pitchMotorp),
      pid(pidConfig)
{
}

void ChassisFramePitchTurretController::initialize()
{
    if (turretMotor.getTurretController() != this)
    {
        pid.reset();
        turretMotor.attachTurretController(this);
    }
}


// float pitchErrorDebug = 0;
// float pitchSetpointDebug = 0;
// float pitchAngleDebug = 0;
// float gravitationalOffsetDebug = 0;
// float pitchPidOutputDebug = 0;
void ChassisFramePitchTurretController::runController(
    const uint32_t dt,
    const float desiredSetpoint)
{
    // limit the yaw min and max angles
    turretMotor.updateMotorAngle();
    turretMotor.setChassisFrameSetpoint(desiredSetpoint);
    // pitchSetpointDebug = desiredSetpoint;

    
    // position controller based on turret pitch gimbal
    // float positionControllerError = turretMotor.getValidMinError(turretMotor.getChassisFrameSetpoint(), turretMotor.getChassisFrameUnwrappedMeasuredAngle()* 3/4);

    float positionControllerError = turretMotor.getValidChassisMeasurementError();


    // pitchErrorDebug = positionControllerError;
    // pitchAngleDebug = turretMotor.getChassisFrameUnwrappedMeasuredAngle();

    float pidOutput =
        pid.runController(positionControllerError, turretMotor.getChassisFrameVelocity(), dt);
    // pitchPidOutputDebug = pidOutput;

        pidOutput += computeGravitationalForceOffset(
        59,
        0,
        -turretMotor.getAngleFromCenter(),
        1600);

    turretMotor.setMotorOutput(pidOutput);
}

void ChassisFramePitchTurretController::setSetpoint(float desiredSetpoint)
{
    turretMotor.setChassisFrameSetpoint(desiredSetpoint);
}

float ChassisFramePitchTurretController::getSetpoint() const
{
    return turretMotor.getChassisFrameSetpoint();
}

float ChassisFramePitchTurretController::getMeasurement() const
{
    return turretMotor.getChassisFrameUnwrappedMeasuredAngle();
}

bool ChassisFramePitchTurretController::isOnline() const { return turretMotor.isOnline(); }

}  // namespace control::turret::algorithms
