#include "turret_double_motor_rev.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/motor/dji_motor.hpp"

using namespace tap::motor;
using namespace tap::algorithms;

namespace src::control::turret
{
TurretDoubleMotorRev::TurretDoubleMotorRev(
    tap::motor::RevMotor *motor1,
    tap::motor::RevMotor *motor2,
    const TurretMotorConfig &motorConfig)
    : config(motorConfig),
      motor1(motor1),
      motor2(motor2),
      ratio(config.ratio),
      chassisFrameSetpoint(Angle(config.startAngle)),
      chassisFrameMeasuredAngle(Angle(config.startAngle))
{
    assert(config.minAngle <= config.maxAngle);
    assert(motor1 != nullptr);
    assert(motor2 != nullptr);
}

void TurretDoubleMotorRev::updateMotorAngle()
{
    if (isOnline())
    {
        float chassisFrameUnwrappedMeasurement =
            motor1->getEncoder()->getPosition().getUnwrappedValue();

        chassisFrameMeasuredAngle.setUnwrappedValue(chassisFrameUnwrappedMeasurement * ratio);
    }
    else
    {
        chassisFrameMeasuredAngle.setUnwrappedValue(config.startAngle);
    }
}
void TurretDoubleMotorRev::setMotorOutput(float out)
{
    out = limitVal(out, -MAX_OUT_REV, MAX_OUT_REV);

    if (motor1->isMotorOnline() && motor2->isMotorOnline())
    {
        motor1->setControlValue(out);
        motor2->setControlValue(out);
    }
    else
    {
        motor1->setControlValue(0);
        motor2->setControlValue(0);
    }
}

void TurretDoubleMotorRev::setChassisFrameSetpoint(WrappedFloat setpoint)
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

float TurretDoubleMotorRev::getValidChassisMeasurementError() const
{
    return getValidMinError(chassisFrameSetpoint, chassisFrameMeasuredAngle);
}

float TurretDoubleMotorRev::getValidMinError(
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
