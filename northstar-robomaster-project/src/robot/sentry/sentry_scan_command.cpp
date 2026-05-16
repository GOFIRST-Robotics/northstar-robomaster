#include "sentry_scan_command.hpp"

namespace src::control::turret::cv
{
SentryScanCommand::SentryScanCommand(
    tap::Drivers *drivers,
    TurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController,
    float DELTA_MAX,
    float MAX_ERROR,
    float ROT_SPEED)
    : drivers(drivers),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      DELTA_MAX(DELTA_MAX),
      MAX_ERROR(MAX_ERROR),
      ROT_SPEED(ROT_SPEED)
{
    addSubsystemRequirement(turretSubsystem);
}

bool SentryScanCommand::isReady() { return !isFinished(); }

void SentryScanCommand::initialize()
{
    yawController->initialize();
    pitchController->initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
    // used to use offsets
}
float debugSetPoint = 0;
void SentryScanCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    float Measurement = yawController->getMeasurement().getUnwrappedValue();
    float SetPoint = yawController->getSetpoint().getUnwrappedValue();
    debugSetPoint = SetPoint;

    yawController->runController(dt, Angle(SetPoint + ROT_SPEED));

    pitchController->runController(dt, Angle(turretSubsystem->pitchMotor.getConfig().startAngle));
}

bool SentryScanCommand::isFinished() const
{
    return !pitchController->isOnline() && !yawController->isOnline();
}

void SentryScanCommand::end(bool)
{
    turretSubsystem->yawMotor.setMotorOutput(0);
    turretSubsystem->pitchMotor.setMotorOutput(0);
}

}  // namespace src::control::turret::cv