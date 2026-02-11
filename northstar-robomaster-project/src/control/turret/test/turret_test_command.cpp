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

#include "turret_test_command.hpp"

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/drivers.hpp"

using tap::algorithms::WrappedFloat;

namespace src::control::turret::test
{
TurretTestCommand::TurretTestCommand(
    TurretSubsystem *turretSubsystem,
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

bool TurretTestCommand::isReady() { return turretSubsystem->yawMotor.isOnline(); }

float totalTime = 0;

void TurretTestCommand::initialize()
{
    startYawAngle = turretSubsystem->yawMotor.getChassisFrameMeasuredAngle();

    startPitchAngle = turretSubsystem->pitchMotor.getChassisFrameMeasuredAngle();

    WrappedFloat newYawSetpoint = startYawAngle + yawMoveAmount;

    turretSubsystem->yawMotor.setChassisFrameSetpoint(newYawSetpoint);

    // turretSubsystem->yawMotor.attachTurretController(nullptr);

    WrappedFloat newPitchSetpoint = startPitchAngle + pitchMoveAmount;

    turretSubsystem->pitchMotor.setChassisFrameSetpoint(newPitchSetpoint);

    // turretSubsystem->pitchMotor.attachTurretController(nullptr);

    startTime = tap::arch::clock::getTimeMilliseconds();
}

float yawAngle;
float pitchAngle;

void TurretTestCommand::execute()
{
    yawAngle =
        abs(turretSubsystem->yawMotor.getChassisFrameMeasuredAngle().getWrappedValue() -
            startYawAngle.getWrappedValue());
    pitchAngle =
        abs(turretSubsystem->pitchMotor.getChassisFrameMeasuredAngle().getWrappedValue() -
            startPitchAngle.getWrappedValue());
}

bool TurretTestCommand::isFinished() const
{
    WrappedFloat yawSetpoint = turretSubsystem->yawMotor.getChassisFrameSetpoint();
    bool yawMovementDone =
        yawSetpoint.minDifference(turretSubsystem->yawMotor.getChassisFrameMeasuredAngle()) <=
        allowedError;

    WrappedFloat pitchSetpoint = turretSubsystem->pitchMotor.getChassisFrameSetpoint();
    bool pitchMovementDone =
        pitchSetpoint.minDifference(turretSubsystem->pitchMotor.getChassisFrameMeasuredAngle()) <=
        allowedError;

    return yawMovementDone && pitchMovementDone;
}

void TurretTestCommand::end(bool)
{
    endTime = tap::arch::clock::getTimeMilliseconds();
    totalTime = (endTime - startTime) / 1000.0f;
}

}  // namespace src::control::turret::test
