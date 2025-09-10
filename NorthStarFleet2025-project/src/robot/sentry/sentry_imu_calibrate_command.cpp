#include "sentry_imu_calibrate_command.hpp"

#include "tap/drivers.hpp"

#include "control/turret/constants/turret_constants.hpp"

using namespace tap::algorithms;
using namespace tap::communication::sensors::imu::bmi088;

namespace src::control::imu
{
SentryImuCalibrateCommand::SentryImuCalibrateCommand(
    tap::Drivers *drivers,
    const std::vector<TurretIMUCalibrationConfig> &turretsAndControllers,
    chassis::ChassisSubsystem *chassis,
    float velocityZeroThreshold,
    float positionZeroThreshold)
    : ImuCalibrateCommandBase(),
      velocityZeroThreshold(velocityZeroThreshold),
      positionZeroThreshold(positionZeroThreshold),
      drivers(drivers),
      turretsAndControllers(turretsAndControllers),
      chassis(chassis)
{
    for (auto &config : turretsAndControllers)
    {
        // assert(config.turretMCBCanComm != nullptr);
        assert(config.turret != nullptr);
        assert(config.yawControllerTop != nullptr);
        assert(config.yawControllerBottom != nullptr);
        assert(config.pitchControllerTop != nullptr);
        assert(config.pitchControllerBottom != nullptr);

        addSubsystemRequirement(config.turret);
    }

    addSubsystemRequirement(chassis);
}

bool SentryImuCalibrateCommand::isReady() { return true; }

void SentryImuCalibrateCommand::initialize()
{
    calibrationState = CalibrationState::WAITING_FOR_SYSTEMS_ONLINE;

    if (chassis != nullptr)
    {
        chassis->setVelocityFieldDrive(0, 0, 0);
    }

    for (auto &config : turretsAndControllers)
    {
        config.turret->yawMotorTop.setChassisFrameSetpoint(
            Angle(config.turret->yawMotorTop.getConfig().startAngle));
        config.turret->pitchMotorTop.setChassisFrameSetpoint(
            Angle(config.turret->pitchMotorTop.getConfig().startAngle));
        config.turret->yawMotorBottom.setChassisFrameSetpoint(
            Angle(config.turret->yawMotorBottom.getConfig().startAngle));
        config.turret->pitchMotorBottom.setChassisFrameSetpoint(
            Angle(config.turret->pitchMotorBottom.getConfig().startAngle));
        config.pitchControllerTop->initialize();
        config.yawControllerTop->initialize();
        config.pitchControllerBottom->initialize();
        config.yawControllerBottom->initialize();
    }

    calibrationLongTimeout.stop();
    calibrationTimer.stop();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void SentryImuCalibrateCommand::execute()
{
    switch (calibrationState)
    {
        case CalibrationState::WAITING_FOR_SYSTEMS_ONLINE:
        {
            // Only start calibrating if the turret is online and if there is an IMU online to be
            // calibrated. The onboard Bmi088 will never be in the `IMU_NOT_CONNECTED` state unless
            // the Bmi088 is shorted (which has never happened). The turret MCB will only be
            // offline if the turret MCB is unplugged.
            bool turretsOnline = true;

            for (auto &config : turretsAndControllers)
            {
                // turretMCBsReady &= config.turretMCBCanComm->isConnected();
                turretsOnline &= config.turret->isOnline();
            }

            if (turretsOnline &&
                (drivers->bmi088.getImuState() != Bmi088::ImuState::IMU_NOT_CONNECTED))
            {
                calibrationLongTimeout.restart(MAX_CALIBRATION_WAITTIME_MS);
                calibrationTimer.restart(WAIT_TIME_TURRET_RESPONSE_MS);
                calibrationState = CalibrationState::LOCKING_TURRET;
            }

            break;
        }
        case CalibrationState::LOCKING_TURRET:
        {
            bool turretsNotMoving = true;
            for (auto &config : turretsAndControllers)
            {
                turretsNotMoving &= turretReachedCenterAndNotMoving(
                    config.turret,
                    !config.turretImuOnTopPitch,
                    !config.turretImuOnBottomPitch);
            }

            if (calibrationTimer.isExpired() && turretsNotMoving)
            {
                // enter calibration phase
                calibrationTimer.stop();

                for (auto &config : turretsAndControllers)
                {
                    // config.turretMCBCanComm->sendImuCalibrationRequest();
                }

                drivers->bmi088.requestCalibration();
                calibrationState = CalibrationState::CALIBRATING_IMU;
            }

            break;
        }
        case CalibrationState::CALIBRATING_IMU:
            if (drivers->bmi088.getImuState() == Bmi088::ImuState::IMU_CALIBRATED)
            {
                // assume turret MCB takes approximately as long as the onboard IMU to calibrate,
                // plus 1 second extra to handle sending the request and processing it
                // TODO to handle the case where the turret MCB doesn't receive information,
                // potentially add ACK sequence to turret MCB CAN comm class.
                calibrationTimer.restart(TURRET_IMU_EXTRA_WAIT_CALIBRATE_MS);
                calibrationState = CalibrationState::BUZZING;
            }
            buzzerTimer.restart(1000);
            break;
        case CalibrationState::BUZZING:
            if (buzzerTimer.isExpired())
            {
                calibrationState = CalibrationState::WAITING_CALIBRATION_COMPLETE;
            }
            tap::buzzer::playNote(&drivers->pwm, notes[4 - buzzerTimer.timeRemaining() / 201]);
            break;
        case CalibrationState::WAITING_CALIBRATION_COMPLETE:
            tap::buzzer::silenceBuzzer(&drivers->pwm);
            break;
    }

    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    for (auto &config : turretsAndControllers)
    {
        // don't run pitch controller when turret IMU not on pitch (as there is no need)
        if (config.turretImuOnTopPitch)
        {
            config.pitchControllerTop->runController(
                dt,
                config.turret->pitchMotorTop.getChassisFrameSetpoint());
        }
        if (config.turretImuOnBottomPitch)
        {
            config.pitchControllerBottom->runController(
                dt,
                config.turret->pitchMotorBottom.getChassisFrameSetpoint());
        }

        config.yawControllerTop->runController(
            dt,
            config.turret->yawMotorTop.getChassisFrameSetpoint());
        config.yawControllerBottom->runController(
            dt,
            config.turret->yawMotorBottom.getChassisFrameSetpoint());
    }
}

void SentryImuCalibrateCommand::end(bool)
{
    for (auto &config : turretsAndControllers)
    {
        config.turret->yawMotorTop.setMotorOutput(0);
        config.turret->pitchMotorTop.setMotorOutput(0);
        config.turret->yawMotorBottom.setMotorOutput(0);
        config.turret->pitchMotorBottom.setMotorOutput(0);
    }
}

bool SentryImuCalibrateCommand::isFinished() const
{
    return (calibrationState == CalibrationState::WAITING_CALIBRATION_COMPLETE &&
            calibrationTimer.isExpired()) ||
           calibrationLongTimeout.isExpired();
}

}  // namespace src::control::imu
