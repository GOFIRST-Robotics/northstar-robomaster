/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "turret_motor.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/motor/dji_motor.hpp"

using namespace tap::motor;
using namespace tap::algorithms;

namespace src::control::turret
{
TurretMotor::TurretMotor(tap::motor::MotorInterface *motor, const TurretMotorConfig &motorConfig)
    : config(motorConfig),
      motor(motor),
      ratio(config.ratio),
      chassisFrameSetpoint(Angle(config.startAngle)),
      chassisFrameMeasuredAngle(Angle(config.startAngle))
{
    assert(config.minAngle <= config.maxAngle);
    assert(motor != nullptr);
}

void TurretMotor::updateMotorAngle()
{
    if (isOnline())
    {
        float chassisFrameUnwrappedMeasurement =
            motor->getEncoder()->getPosition().getUnwrappedValue();

        chassisFrameMeasuredAngle.setUnwrappedValue(chassisFrameUnwrappedMeasurement * ratio);
    }
    else
    {
        chassisFrameMeasuredAngle.setUnwrappedValue(config.startAngle);
    }
}

void TurretMotor::setMotorOutput(float out)
{
    out = limitVal(out, -MAX_OUT_6020, MAX_OUT_6020);

    if (motor->isMotorOnline())
    {
        motor->setDesiredOutput(out);
    }
    else
    {
        motor->setDesiredOutput(0);
    }
}

void TurretMotor::setChassisFrameSetpoint(WrappedFloat setpoint)
{
    chassisFrameSetpoint = setpoint;

    if (config.limitMotorAngles)
    {
        int status;
        chassisFrameSetpoint = Angle(WrappedFloat::limitValue(
            chassisFrameSetpoint,
            config.minAngle,
            config.maxAngle,
            &status));
    }
}

float TurretMotor::getValidChassisMeasurementError() const
{
    return getValidMinError(chassisFrameSetpoint, chassisFrameMeasuredAngle);
}

float TurretMotor::getValidMinError(const WrappedFloat setpoint, const WrappedFloat measurement)
    const
{
    if (config.limitMotorAngles)
    {
        float pos = WrappedFloat::rangeOverlap(
            measurement,
            setpoint,
            Angle(config.maxAngle),
            Angle(config.minAngle));
        float neg = WrappedFloat::rangeOverlap(
            setpoint,
            measurement,
            Angle(config.maxAngle),
            Angle(config.minAngle));

        if (pos < neg)
        {
            return (setpoint - measurement).getWrappedValue();
        }
        else if (pos > neg)
        {
            return (setpoint - measurement).getWrappedValue() - static_cast<float>(M_TWOPI);
        }
    }

    // the error can be wrapped around the unit circle
    // equivalent to this - other
    return measurement.minDifference(setpoint);
}

}  // namespace src::control::turret
