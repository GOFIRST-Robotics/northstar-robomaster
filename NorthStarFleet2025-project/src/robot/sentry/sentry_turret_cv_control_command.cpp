#include "sentry_turret_cv_control_command.hpp"

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
    algorithms::TurretYawControllerInterface *yawControllerTop,
    algorithms::TurretPitchControllerInterface *pitchControllerTop,
    algorithms::TurretYawControllerInterface *yawControllerBottom,
    algorithms::TurretPitchControllerInterface *pitchControllerBottom,
    float userYawInputScalar,
    float userPitchInputScalar,
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
}

void SentryTurretCVControlCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;
    WrappedFloat pitchSetpointBottom = Angle(0);
    WrappedFloat yawSetpointBottom = Angle(0);
    WrappedFloat pitchSetpointTop = Angle(0);

    if (visionComms.isAimDataUpdated(bottomID))
    {
        // up has positive error so up positive
        pitchSetpointBottom = Angle(
            pitchControllerBottom->getMeasurement().getUnwrappedValue() -
            limitVal(visionComms.getLastAimData(bottomID).pitch, -0.1f, 0.1f));
        pitchControllerBottom->runController(dt, pitchSetpointBottom);
        // left neg right post
        yawSetpointBottom = Angle(
            -yawControllerBottom->getMeasurement().getUnwrappedValue() +
            visionComms.getLastAimData(bottomID).yaw);
        yawControllerBottom->runController(dt, yawSetpointBottom);

        bottomWithinAimingTolerance =  // TODO calculate off the distance
            (abs(visionComms.getLastAimData(bottomID).yaw) < AIMING_TOLERANCE_YAW &&
             abs(visionComms.getLastAimData(bottomID).pitch) < AIMING_TOLERANCE_PITCH);
    }
    else
    {
        pitchSetpointBottom =
            pitchControllerBottom->getSetpoint() +
            userPitchInputScalar * controlOperatorInterface.getTurretPitchInput(bottomID);
        pitchControllerBottom->runController(dt, pitchSetpointBottom);

        yawSetpointBottom =
            yawControllerBottom->getSetpoint() +
            userYawInputScalar * controlOperatorInterface.getTurretYawInput(bottomID);
        yawControllerBottom->runController(dt, yawSetpointBottom);

        bottomWithinAimingTolerance = false;
    }
    if (visionComms.isAimDataUpdated(topID))
    {
        pitchSetpointTop = Angle(
            pitchControllerTop->getMeasurement().getUnwrappedValue() -
            limitVal(visionComms.getLastAimData(topID).pitch, -0.1f, 0.1f));
        pitchControllerTop->runController(dt, pitchSetpointTop);

        float bottomMeasurement = yawControllerBottom->getMeasurementMotor().getUnwrappedValue() -
                                  bottomMeasurementOffset;

        float delta =
            -(yawControllerTop->getMeasurement().getUnwrappedValue() - topMeasurementOffset);

        float input = userPitchInputScalar * controlOperatorInterface.getTurretYawInput(topID) +
                      visionComms.getLastAimData(topID).yaw;

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

        yawControllerTop->runController(dt, Angle(yawSetpointTop));
    }
    else
    {
        pitchSetpointTop =
            pitchControllerTop->getSetpoint() +
            userPitchInputScalar * controlOperatorInterface.getTurretPitchInput(topID);
        pitchControllerTop->runController(dt, pitchSetpointTop);

        float bottomMeasurement = yawControllerBottom->getMeasurementMotor().getUnwrappedValue() -
                                  bottomMeasurementOffset;

        float delta =
            -(yawControllerTop->getMeasurement().getUnwrappedValue() - topMeasurementOffset);

        float input = userPitchInputScalar * controlOperatorInterface.getTurretYawInput(topID);

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

        yawControllerTop->runController(dt, Angle(yawSetpointTop));
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
