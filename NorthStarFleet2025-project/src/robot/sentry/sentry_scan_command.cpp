#include "sentry_scan_command.hpp"

namespace src::control::turret::cv
{
SentryScanCommand::SentryScanCommand(
    tap::Drivers *drivers,
    SentryTurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawControllerBottom,
    algorithms::TurretPitchControllerInterface *pitchControllerBottom,
    algorithms::TurretYawControllerInterface *yawControllerTop,
    algorithms::TurretPitchControllerInterface *pitchControllerTop,
    float DELTA_MAX,
    float MAX_ERROR,
    float ROT_SPEED)
    : drivers(drivers),
      turretSubsystem(turretSubsystem),
      yawControllerBottom(yawControllerBottom),
      pitchControllerBottom(pitchControllerBottom),
      yawControllerTop(yawControllerTop),
      pitchControllerTop(pitchControllerTop),
      DELTA_MAX(DELTA_MAX),
      MAX_ERROR(MAX_ERROR),
      ROTATION_SPEED(ROT_SPEED)
{
    addSubsystemRequirement(turretSubsystem);
}

bool SentryScanCommand::isReady() { return !isFinished(); }

void SentryScanCommand::initialize()
{
    yawControllerBottom->initialize();
    pitchControllerBottom->initialize();
    yawControllerTop->initialize();
    pitchControllerTop->initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
    if (!turretSubsystem->offsets)
    {
        turretSubsystem->setOffsets(
            yawControllerBottom->getSetpoint().getUnwrappedValue(),
            yawControllerBottom->getMeasurement().getUnwrappedValue(),
            yawControllerTop->getMeasurement().getUnwrappedValue());
    }
}
float debugbottomSetPoint = 0;
void SentryScanCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    float bottomMeasurement = yawControllerBottom->getMeasurement().getUnwrappedValue() -
                              turretSubsystem->bottomMeasurementOffset;
    float topMeasurement = yawControllerTop->getMeasurement().getUnwrappedValue() -
                           turretSubsystem->topMeasurementOffset;
    float bottomSetPoint = yawControllerBottom->getSetpoint().getUnwrappedValue();
    debugbottomSetPoint = bottomSetPoint;
    float topSetPoint = yawControllerTop->getSetpoint().getUnwrappedValue();

    yawControllerBottom->runController(dt, Angle(bottomSetPoint + ROTATION_SPEED));

    pitchControllerBottom->runController(
        dt,
        Angle(turretSubsystem->pitchMotorBottom.getConfig().startAngle));

    float error = limitVal(float(M_PI - topSetPoint), -MAX_ERROR, MAX_ERROR);
    if (topSetPoint > M_PI - .01 && topSetPoint < M_PI + .01)
    {
        error = M_PI - topSetPoint;
    }

    yawControllerTop->runController(dt, Angle(topSetPoint + error));

    pitchControllerTop->runController(
        dt,
        Angle(turretSubsystem->pitchMotorTop.getConfig().startAngle));
}

bool SentryScanCommand::isFinished() const
{
    return !pitchControllerBottom->isOnline() && !yawControllerBottom->isOnline() &&
           !pitchControllerTop->isOnline() && !yawControllerTop->isOnline();
}

void SentryScanCommand::end(bool)
{
    turretSubsystem->yawMotorBottom.setMotorOutput(0);
    turretSubsystem->pitchMotorBottom.setMotorOutput(0);
    turretSubsystem->yawMotorTop.setMotorOutput(0);
    turretSubsystem->pitchMotorTop.setMotorOutput(0);
}

}  // namespace src::control::turret::cv