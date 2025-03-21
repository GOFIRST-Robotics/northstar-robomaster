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

#include "turret_user_control_command.hpp"

#include "tap/drivers.hpp"

#include "robot/control_operator_interface.hpp"

namespace control::turret::user
{
TurretUserControlCommand::TurretUserControlCommand(
    tap::Drivers *drivers,
    src::control::ControlOperatorInterface* controlOperatorInterface,
    src::can::TurretMCBCanComm *turretMCBCanComm,
    TurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController,
    float userYawInputScalar,
    float userPitchInputScalar,
    uint8_t turretID)
    : drivers(drivers),
      controlOperatorInterface(controlOperatorInterface),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      userYawInputScalar(userYawInputScalar),
      userPitchInputScalar(userPitchInputScalar),
      turretID(turretID),
      turretMCBCanComm(turretMCBCanComm)
{
    addSubsystemRequirement(turretSubsystem);
}

bool TurretUserControlCommand::isReady() { return !isFinished(); }

void TurretUserControlCommand::initialize()
{
    yawController->initialize();
    pitchController->initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}


// float pitchSetpointDebug = 0;
float angle = 6;
float lastYaw;
float debug = 0;
void TurretUserControlCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    const float pitchSetpoint =
        pitchController->getSetpoint() +
        userPitchInputScalar * controlOperatorInterface->getTurretPitchInput(turretID);
    // pitchSetpointDebug = pitchSetpoint;

    pitchController->runController(dt, pitchSetpoint);

    drivers->bmi088.read();
    const float yawSetpoint = -(turretSubsystem->turretGyro.getYaw()-lastYaw) +
        yawController->getSetpoint() +
        userYawInputScalar * controlOperatorInterface->getTurretYawInput(turretID);
    // angle = -turretSubsystem->turretGyro.getYaw();
    // const float yawSetpoint = yawController->getSetpoint() + angle;
    yawController->runController(dt, yawSetpoint);
    drivers->bmi088.read();
    lastYaw = turretSubsystem->turretGyro.getYaw();
    debug = turretMCBCanComm->getYaw();
}

bool TurretUserControlCommand::isFinished() const
{
    return !pitchController->isOnline() && !yawController->isOnline();
    // return !pitchController->isOnline();
    // return !yawController->isOnline();
}

void TurretUserControlCommand::end(bool)
{
    turretSubsystem->yawMotor.setMotorOutput(0);
    turretSubsystem->pitchMotor.setMotorOutput(0);
}

}  // namespace control::turret::user
