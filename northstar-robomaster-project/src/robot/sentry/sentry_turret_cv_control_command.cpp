#ifdef TARGET_SENTRY

#include "sentry_turret_cv_control_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/wrapped_float.hpp"
#include "tap/drivers.hpp"

#include "robot/control_operator_interface.hpp"
#include "robot/sentry/sentry_turret_subsystem.hpp"

using tap::algorithms::WrappedFloat;

namespace src::control::turret::cv
{
SentryTurretCVControlCommand::SentryTurretCVControlCommand(
    tap::Drivers *drivers,
    ControlOperatorInterface &controlOperatorInterface,
    src::serial::VisionComms &visionComms,
    SentryTurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawControllerBottom,
    algorithms::TurretPitchControllerInterface *pitchControllerBottom,
    algorithms::TurretYawControllerInterface *yawControllerTop,
    algorithms::TurretPitchControllerInterface *pitchControllerTop,
    float userYawInputScalar,
    float userPitchInputScalar,
    float MAX_ERROR,
    float DELTA_MAX)
    : drivers(drivers),
      controlOperatorInterface(controlOperatorInterface),
      visionComms(visionComms),
      turretSubsystem(turretSubsystem),
      yawControllerTop(yawControllerTop),
      pitchControllerTop(pitchControllerTop),
      yawControllerBottom(yawControllerBottom),
      pitchControllerBottom(pitchControllerBottom),
      userYawInputScalar(userYawInputScalar),
      userPitchInputScalar(userPitchInputScalar),
      MAX_ERROR(MAX_ERROR),
      DELTA_MAX(DELTA_MAX)
{
    TurretCVControlCommandTemplate::addSubsystemRequirement(turretSubsystem);
}
bool SentryTurretCVControlCommand::isReady() { return !isFinished(); }

void SentryTurretCVControlCommand::initialize()
{
    yawControllerTop->initialize();
    pitchControllerTop->initialize();
    yawControllerBottom->initialize();
    pitchControllerBottom->initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
    drivers->leds.set(tap::gpio::Leds::Green, true);

    yawSetpointTop = yawControllerTop->getSetpoint().getUnwrappedValue();
    if (!turretSubsystem->offsets)
    {
        turretSubsystem->setOffsets(
            yawControllerBottom->getSetpoint().getUnwrappedValue(),
            yawControllerBottom->getMeasurement().getUnwrappedValue(),
            yawControllerTop->getMeasurement().getUnwrappedValue());
    }
    comp = yawControllerTop->getSetpoint().getUnwrappedValue() +
           yawControllerBottom->getMeasurement().getUnwrappedValue() -
           turretSubsystem->bottomMeasurementOffset;
}

void SentryTurretCVControlCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;
    WrappedFloat pitchSetpointBottom = Angle(0);
    WrappedFloat yawSetpointBottom = Angle(0);
    WrappedFloat pitchSetpointTop = Angle(0);

    if (visionComms.isAimDataUpdated(1))
    {
        // up has positive error so up positive
        pitchSetpointBottom = Angle(
            pitchControllerBottom->getMeasurement().getUnwrappedValue() -
            limitVal(visionComms.getLastAimData(1).pitch, -0.1f, 0.1f));
        pitchControllerBottom->runController(dt, pitchSetpointBottom);
        // left neg right post
        yawSetpointBottom = Angle(
            -yawControllerBottom->getMeasurement().getUnwrappedValue() +
            visionComms.getLastAimData(1).yaw);

        bottomWithinAimingTolerance =
            (abs(visionComms.getLastAimData(1).yaw) < visionComms.getLastAimData(1).maxErrorYaw &&
             abs(visionComms.getLastAimData(1).pitch) <
                 visionComms.getLastAimData(1).maxErrorPitch);

        yawControllerBottom->runController(dt, yawSetpointBottom);
    }
    else
    {
        if (visionComms.isAimDataUpdated(0))
        {
            pitchControllerBottom->runController(
                dt,
                Angle(turretSubsystem->pitchMotorBottom.getConfig().startAngle));

            float topSetPoint = yawControllerTop->getSetpoint().getUnwrappedValue();
            float error = limitVal(
                yawControllerBottom->getSetpoint().getUnwrappedValue() - topSetPoint + comp,
                -MAX_ERROR,
                MAX_ERROR);
            if (yawControllerBottom->getSetpoint().getUnwrappedValue() - topSetPoint + comp >
                    -.01 &&
                yawControllerBottom->getSetpoint().getUnwrappedValue() - topSetPoint + comp < .01)
            {
                error =
                    -yawControllerBottom->getSetpoint().getUnwrappedValue() - topSetPoint + comp;
            }

            yawControllerBottom->runController(
                dt,
                Angle(yawControllerBottom->getSetpoint().getUnwrappedValue() + error));
        }
        else
        {
            pitchSetpointBottom =
                pitchControllerBottom->getSetpoint() +
                userPitchInputScalar * controlOperatorInterface.getTurretPitchInput(1);
            pitchControllerBottom->runController(dt, pitchSetpointBottom);

            yawSetpointBottom = yawControllerBottom->getSetpoint() +
                                userYawInputScalar * controlOperatorInterface.getTurretYawInput(1);
            yawControllerBottom->runController(dt, yawSetpointBottom);
        }
        bottomWithinAimingTolerance = false;
    }
    if (visionComms.isAimDataUpdated(0))
    {
        pitchSetpointTop = Angle(
            pitchControllerTop->getMeasurement().getUnwrappedValue() -
            limitVal(visionComms.getLastAimData(0).pitch, -0.1f, 0.1f));
        pitchControllerTop->runController(dt, pitchSetpointTop);

        float bottomMeasurement = yawControllerBottom->getMeasurementMotor().getUnwrappedValue() -
                                  bottomMeasurementOffset;

        float delta =
            -(yawControllerTop->getMeasurement().getUnwrappedValue() - topMeasurementOffset);

        float input = userPitchInputScalar * controlOperatorInterface.getTurretYawInput(0) +
                      visionComms.getLastAimData(0).yaw;

        if (delta <= -DELTA_MAX)
        {
            comp = bottomMeasurement + DELTA_MAX;
        }
        else if (delta >= DELTA_MAX)
        {
            comp = bottomMeasurement - DELTA_MAX;
        }

        if (yawSetpointTop + input < DELTA_MAX && yawSetpointTop + input > -DELTA_MAX)
        {
            comp += input;
        }
        else if (input != 0)
        {
            comp = getSign(input) * DELTA_MAX + yawSetpointBottom.getUnwrappedValue() -
                   bottomSetpointOffset;
        }

        yawSetpointTop = limitVal(
            -(yawSetpointBottom.getUnwrappedValue() - bottomSetpointOffset) + comp,
            -DELTA_MAX,
            DELTA_MAX);

        if (abs(yawSetpointTop) == DELTA_MAX && input != 0 && yawSetpointTop + input < DELTA_MAX &&
            yawSetpointTop + input > -DELTA_MAX)
        {
            yawSetpointTop += input;
        }

        topWithinAimingTolerance =
            (abs(visionComms.getLastAimData(0).yaw) < visionComms.getLastAimData(0).maxErrorYaw &&
             abs(visionComms.getLastAimData(0).pitch) <
                 visionComms.getLastAimData(0).maxErrorPitch);

        yawControllerTop->runController(dt, Angle(yawSetpointTop));
    }
    else
    {
        if (visionComms.isAimDataUpdated(1))
        {
            pitchControllerTop->runController(
                dt,
                Angle(turretSubsystem->pitchMotorTop.getConfig().startAngle));

            float topSetPoint = yawControllerTop->getSetpoint().getUnwrappedValue();

            float error = limitVal(float(-topSetPoint), -MAX_ERROR, MAX_ERROR);
            if (topSetPoint > -.01 && topSetPoint < .01)
            {
                error = -topSetPoint;
            }

            yawControllerTop->runController(dt, Angle(topSetPoint + error));
        }
        else
        {
            pitchSetpointTop =
                pitchControllerTop->getSetpoint() +
                userPitchInputScalar * controlOperatorInterface.getTurretPitchInput(0);
            pitchControllerTop->runController(dt, pitchSetpointTop);

            float bottomMeasurement =
                yawControllerBottom->getMeasurementMotor().getUnwrappedValue() -
                bottomMeasurementOffset;

            float delta =
                -(yawControllerTop->getMeasurement().getUnwrappedValue() - topMeasurementOffset);

            float input = userPitchInputScalar * controlOperatorInterface.getTurretYawInput(0);

            if (delta <= -DELTA_MAX)
            {
                comp = bottomMeasurement + DELTA_MAX;
            }
            else if (delta >= DELTA_MAX)
            {
                comp = bottomMeasurement - DELTA_MAX;
            }

            if (yawSetpointTop + input < DELTA_MAX && yawSetpointTop + input > -DELTA_MAX)
            {
                comp += input;
            }
            else if (input != 0)
            {
                comp = getSign(input) * DELTA_MAX + yawSetpointBottom.getUnwrappedValue() -
                       bottomSetpointOffset;
            }

            yawSetpointTop = limitVal(
                -(yawSetpointBottom.getUnwrappedValue() - bottomSetpointOffset) + comp,
                -DELTA_MAX,
                DELTA_MAX);

            if (abs(yawSetpointTop) == DELTA_MAX && input != 0 &&
                yawSetpointTop + input < DELTA_MAX && yawSetpointTop + input > -DELTA_MAX)
            {
                yawSetpointTop += input;
            }

            yawControllerTop->runController(dt, Angle(yawSetpointTop));
        }
        topWithinAimingTolerance = false;
    }
}

bool SentryTurretCVControlCommand::isFinished() const
{
    return !pitchControllerBottom->isOnline() && !yawControllerBottom->isOnline() &&
           yawControllerTop->isOnline() && pitchControllerTop->isOnline();
}

void SentryTurretCVControlCommand::end(bool interrupted)
{
    turretSubsystem->yawMotorBottom.setMotorOutput(0);
    turretSubsystem->pitchMotorBottom.setMotorOutput(0);
    turretSubsystem->yawMotorTop.setMotorOutput(0);
    turretSubsystem->pitchMotorTop.setMotorOutput(0);

    drivers->leds.set(tap::gpio::Leds::Green, false);
}

}  // namespace src::control::turret::cv

#endif  // TARGET_SENTRY
