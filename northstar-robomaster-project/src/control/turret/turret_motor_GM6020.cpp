#include "turret_motor_GM6020.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/motor/dji_motor.hpp"

using namespace tap::motor;
using namespace tap::algorithms;

namespace src::control::turret
{
TurretMotorGM6020::TurretMotorGM6020(
    tap::motor::MotorInterface *motor,
    const TurretMotorConfig &motorConfig)
    : config(motorConfig),
      motor(motor),
      ratio(config.ratio),
      chassisFrameSetpoint(Angle(config.startAngle)),
      chassisFrameMeasuredAngle(Angle(config.startAngle))
{
    assert(config.minAngle <= config.maxAngle);
    assert(motor != nullptr);
}

void TurretMotorGM6020::updateMotorAngle()
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

void TurretMotorGM6020::setMotorOutput(float out)
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

void TurretMotorGM6020::setChassisFrameSetpoint(WrappedFloat setpoint)
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

float TurretMotorGM6020::getValidChassisMeasurementError() const
{
    return getValidMinError(chassisFrameSetpoint, chassisFrameMeasuredAngle);
}

float TurretMotorGM6020::getValidMinError(
    const WrappedFloat setpoint,
    const WrappedFloat measurement) const
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
