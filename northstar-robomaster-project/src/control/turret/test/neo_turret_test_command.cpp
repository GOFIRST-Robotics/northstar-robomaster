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

#include "neo_turret_test_command.hpp"

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/drivers.hpp"

using tap::algorithms::WrappedFloat;

namespace src::control::turret::test
{
NeoTurretTestCommand::NeoTurretTestCommand(
    RevTurretSubsystem *turretSubsystem,
    float yawMoveAmount,
    float pitchMoveAmount,
    float allowedError)
    : turretSubsystem(turretSubsystem),
      yawMoveAmount(yawMoveAmount),
      pitchMoveAmount(pitchMoveAmount),
      allowedError(allowedError)
{
    addSubsystemRequirement(turretSubsystem);
}

bool NeoTurretTestCommand::isReady() { return turretSubsystem->yawMotor.isOnline(); }

float totalTime = 0;

void NeoTurretTestCommand::initialize()
{
    startYawAngle = turretSubsystem->yawMotor.getChassisFrameMeasuredAngle();

    startPitchAngle = turretSubsystem->pitchMotor.getChassisFrameMeasuredAngle();

    WrappedFloat newYawSetpoint = startYawAngle + yawMoveAmount;

    turretSubsystem->yawMotor.setChassisFrameSetpoint(newYawSetpoint);

    turretSubsystem->yawMotor.attachTurretController(nullptr);

    WrappedFloat newPitchSetpoint = startPitchAngle + pitchMoveAmount;

    turretSubsystem->pitchMotor.setChassisFrameSetpoint(newPitchSetpoint);

    turretSubsystem->pitchMotor.attachTurretController(nullptr);

    startTime = tap::arch::clock::getTimeMilliseconds();
}

float yawAngle;
float pitchAngle;

void NeoTurretTestCommand::execute()
{
    yawAngle =
        abs(turretSubsystem->yawMotor.getChassisFrameMeasuredAngle().getWrappedValue() -
            startYawAngle.getWrappedValue());
    pitchAngle =
        abs(turretSubsystem->pitchMotor.getChassisFrameMeasuredAngle().getWrappedValue() -
            startPitchAngle.getWrappedValue());
}

bool NeoTurretTestCommand::isFinished() const
{
    bool yawMovementDone =
        abs(turretSubsystem->yawMotor.getChassisFrameSetpoint().getWrappedValue() -
            turretSubsystem->yawMotor.getChassisFrameMeasuredAngle().getWrappedValue()) <=
        allowedError;
    bool pitchMovementDone =
        abs(turretSubsystem->pitchMotor.getChassisFrameSetpoint().getWrappedValue() -
            turretSubsystem->pitchMotor.getChassisFrameMeasuredAngle().getWrappedValue()) <=
        allowedError;
    return yawMovementDone && pitchMovementDone;
}

void NeoTurretTestCommand::end(bool)
{
    endTime = tap::arch::clock::getTimeMilliseconds();
    totalTime = (endTime - startTime) / 1000.0f;
}

}  // namespace src::control::turret::test
