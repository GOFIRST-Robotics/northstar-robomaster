#include "sentry_cv_manager_command.hpp"

#include "tap/drivers.hpp"

namespace src::control::turret::cv
{
SentryCvManagerCommand::SentryCvManagerCommand(
    tap::Drivers *drivers,
    src::control::ControlOperatorInterface &controlOperatorInterface,
    src::serial::VisionComms &visionComms,
    src::control::turret::SentryTurretSubsystem *sentryTurretSubsystem,
    src::control::turret::algorithms::TurretYawControllerInterface *yawControllerBottom,
    src::control::turret::algorithms::TurretPitchControllerInterface *pitchControllerBottom,
    src::control::turret::algorithms::TurretYawControllerInterface *yawControllerTop,
    src::control::turret::algorithms::TurretPitchControllerInterface *pitchControllerTop,
    float userYawInputScalar,
    float userPitchInputScalar,
    float DELTA_MAX,
    float MAX_ERROR,
    float ROT_SPEED)
    : tap::control::ComprisedCommand(drivers),
      visionComms(visionComms),
      turretCVControlCommand(
          drivers,
          controlOperatorInterface,
          visionComms,
          sentryTurretSubsystem,
          yawControllerBottom,
          pitchControllerBottom,
          yawControllerTop,  // controler for top turret
          pitchControllerTop,
          userYawInputScalar,
          userPitchInputScalar,
          MAX_ERROR,
          DELTA_MAX),  // +- offset max rads
      turretScanCommand(
          drivers,
          sentryTurretSubsystem,
          yawControllerBottom,
          pitchControllerBottom,
          yawControllerTop,  // controler for top turret
          pitchControllerTop,
          DELTA_MAX,
          MAX_ERROR,
          ROT_SPEED)

{
    comprisedCommandScheduler.registerSubsystem(sentryTurretSubsystem);
    addSubsystemRequirement(sentryTurretSubsystem);
}

bool SentryCvManagerCommand::isReady()
{
    return turretScanCommand.isReady() || turretCVControlCommand.isReady();
}

void SentryCvManagerCommand::initialize()
{
    comprisedCommandScheduler.addCommand(&turretScanCommand);
}
void SentryCvManagerCommand::execute()
{
    if (!comprisedCommandScheduler.isCommandScheduled(&turretCVControlCommand) &&
        (visionComms.isAimDataUpdated(0) || visionComms.isAimDataUpdated(1)))
    {
        comprisedCommandScheduler.addCommand(&turretCVControlCommand);
    }
    if (!comprisedCommandScheduler.isCommandScheduled(&turretScanCommand) &&
        (!visionComms.isAimDataUpdated(0) && !visionComms.isAimDataUpdated(1)))

    {
        comprisedCommandScheduler.addCommand(&turretScanCommand);
    }

    comprisedCommandScheduler.run();
}

bool SentryCvManagerCommand::isFinished() const
{
    return turretCVControlCommand.isFinished() && turretScanCommand.isFinished();
}

void SentryCvManagerCommand::end(bool interrupted)
{
    comprisedCommandScheduler.removeCommand(&turretScanCommand, interrupted);
    comprisedCommandScheduler.removeCommand(&turretCVControlCommand, interrupted);
}

}  // namespace src::control::turret::cv
