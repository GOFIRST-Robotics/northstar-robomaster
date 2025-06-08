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

#include "turret_cv_control_command.hpp"

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/drivers.hpp"

#include "../turret_subsystem.hpp"
#include "robot/control_operator_interface.hpp"

using tap::algorithms::WrappedFloat;

namespace src::control::turret::cv
{
TurretCVControlCommand::TurretCVControlCommand(
    tap::Drivers *drivers,
    src::serial::VisionComms &visionComms,
    TurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController,
    float userYawInputScalar,
    float userPitchInputScalar,
    uint8_t turretID)
    : drivers(drivers),
      visionComms(visionComms),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      userYawInputScalar(userYawInputScalar),
      userPitchInputScalar(userPitchInputScalar),
      turretID(turretID)
{
    addSubsystemRequirement(turretSubsystem);
}
bool TurretCVControlCommand::isReady()
{
    return !isFinished();  //&& this->yawController->isOnline();  // TODO hero needs comented out
}

void TurretCVControlCommand::initialize()
{
    yawController->initialize();
    pitchController->initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void TurretCVControlCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    const WrappedFloat pitchSetpoint =
        pitchController->getMeasurement() + visionComms.getLastAimData(0).pitch;
    pitchController->runController(dt, pitchSetpoint);

    const WrappedFloat yawSetpoint =
        yawController->getMeasurement() + visionComms.getLastAimData(0).yaw;
    yawController->runController(dt, yawSetpoint);
}
bool debugpitchController = false;
bool debugyawController = false;

bool TurretCVControlCommand::isFinished() const
{
    debugpitchController = pitchController->isOnline();
    debugyawController = yawController->isOnline();
    return !pitchController->isOnline() && !yawController->isOnline() ||
           !visionComms.isCvOnline();  //&& TODO not shure if this is right
}

void TurretCVControlCommand::end(bool)
{
    turretSubsystem->yawMotor.setMotorOutput(0);
    turretSubsystem->pitchMotor.setMotorOutput(0);
}

}  // namespace src::control::turret::cv
