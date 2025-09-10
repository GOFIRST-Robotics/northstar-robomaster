#ifndef SENTRY_IMU_CALIBRATE_COMMAND_HPP_
#define SENTRY_IMU_CALIBRATE_COMMAND_HPP_

#include <vector>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "control/chassis/chassis_subsystem.hpp"
#include "control/imu/imu_calibrate_template.hpp"
#include "control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "robot/sentry/sentry_turret_subsystem.hpp"

using namespace tap::algorithms;

namespace src::control::imu
{
/**
 * A command whose job is to perform a calibration of the turret and chassis IMUs. Requires that the
 * robot has a turret and a chassis subsystem. Also requires that a turret IMU is connected via the
 * TurretMCBCanComm object.
 *
 * When this command is scheduled, it performs the following actions:
 * 1. Wait until the turret is online and either the chassis mpu6500 or turret MCB IMU is online.
 * 2. Command the pitch and yaw turret gimbals to move to PI/2 radians (forward and flat).
 * 3. Command the chassis to stay still.
 * 4. Pause until the chassis/turret subsystems are no longer moving.
 * 5. Send a calibration signal to the turret MCB.
 * 6. Send signal to onboard IMU to recalibrate.
 * 7. Wait until calibration is complete and then end the command.
 */
class SentryImuCalibrateCommand : public ImuCalibrateCommandBase
{
public:
    /**
     * Specifies the current calibration state that command is in.
     */
    enum class CalibrationState
    {
        /** While in this state, the command waits for the turret to be online and the IMUs to be
           online. */
        WAITING_FOR_SYSTEMS_ONLINE,
        /** While in this state, the command "locks" the turret at PI/2 radians (horizontal to the
           ground). The command then sends a calibration request to the mpu6500 and the
           TurretMCBCanComm class. */
        LOCKING_TURRET,
        /** While in this state, the command waits until calibration of the IMUs are complete. */
        CALIBRATING_IMU,
        /** While in this state, turn on buzzer so people know we are done*/
        BUZZING,
        /** While in this state, the command waits a small time after calibration is complete to
           handle any latency associated with sending messages to the TurretMCBCanComm. */
        WAITING_CALIBRATION_COMPLETE,
    };

    /**
     * Threshold around 0 where turret pitch and yaw velocity is considered to be 0, in radians/s
     */
    const float velocityZeroThreshold;
    /**
     * Threshold around 0 where turret pitch and yaw position from the center considered to be 0,
     * in radians
     */
    const float positionZeroThreshold;

    static constexpr float DEFAULT_VELOCITY_ZERO_THRESHOLD = modm::toRadian(1e-2);
    static constexpr float DEFAULT_POSITION_ZERO_THRESHOLD = modm::toRadian(3.0f);

    struct TurretIMUCalibrationConfig
    {
        /// The turret mounted IMU to be calibrated.
        // src::can::TurretMCBCanComm *turretMCBCanComm;
        /// A `SentryTurretSubsystem` that this command will control (will lock the turret).
        turret::SentryTurretSubsystem *turret;
        /// A chassis relative yaw controller used to lock the turret.
        turret::algorithms::ChassisFrameYawTurretController *yawControllerTop;
        /// A chassis relative pitch controller used to lock the turret.
        turret::algorithms::ChassisFramePitchTurretController *pitchControllerTop;
        /// A chassis relative yaw controller used to lock the turret.
        turret::algorithms::ChassisFrameYawTurretController *yawControllerBottom;
        /// A chassis relative pitch controller used to lock the turret.
        turret::algorithms::ChassisFramePitchTurretController *pitchControllerBottom;

        /**
         * `true` if the turret IMU is mounted on the Top pitch axis of the
         * turret. In this case the pitch controller doesn't have to reach the horizontal setpoint
         * before calibration is performed.
         */
        bool turretImuOnTopPitch;

        /**
         * `true` if the turret IMU is mounted on the Bottom pitch axis of the
         * turret. In this case the pitch controller doesn't have to reach the horizontal setpoint
         * before calibration is performed.
         */
        bool turretImuOnBottomPitch;
    };

    /**
     * @param[in] drivers A pointer to the global drivers object.
     * @param[in] turretsAndControllers A list of TurretIMUCalibrationConfig structs containing
     * turret and turret IMU information necessary for calibrating the IMU
     * @param[in] chassis A `ChassisSubsystem` that this command will control (will set the desired
     * movement to 0).
     * @param[in] velocityZeroThreshold Threshold around 0 where turret pitch and yaw velocity is
     * considered to be 0, in radians/s.
     * @param[in] positionZeroThreshold Threshold around 0 where turret pitch and yaw position from
     * the center considered to be 0, in radians.
     */
    SentryImuCalibrateCommand(
        tap::Drivers *drivers,
        const std::vector<TurretIMUCalibrationConfig> &turretsAndControllers,
        chassis::ChassisSubsystem *chassis,
        float velocityZeroThreshold = SentryImuCalibrateCommand::DEFAULT_VELOCITY_ZERO_THRESHOLD,
        float positionZeroThreshold = SentryImuCalibrateCommand::DEFAULT_POSITION_ZERO_THRESHOLD);

    const char *getName() const override { return "Calibrate IMU"; }

    bool isReady() override;

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    /**
     * @return The current calibration state of the command.
     */
    CalibrationState getCalibrationState() const { return calibrationState; }

protected:
    /**
     * Wait a minimum of this time to allow the turret to settle at a locked position (in ms).
     */
    static constexpr uint32_t WAIT_TIME_TURRET_RESPONSE_MS = 2000;
    /**
     * Wait this time after the mpu6500 is done calibrating to ensure the turret MCB's IMU is
     * calibrated.
     */
    static constexpr uint32_t TURRET_IMU_EXTRA_WAIT_CALIBRATE_MS = 2000;
    /**
     * Wait timeout (after state `WAITING_FOR_SYSTEMS_ONLINE` is complete) for the command to wait
     * until it gives up. Should never (and has never) happen but is a safety precaution to avoid
     * getting stuck in calibration forever.
     */
    static constexpr uint32_t MAX_CALIBRATION_WAITTIME_MS = 20000;

    tap::Drivers *drivers;
    std::vector<TurretIMUCalibrationConfig> turretsAndControllers;
    chassis::ChassisSubsystem *chassis;

    CalibrationState calibrationState;

    uint32_t prevTime = 0;

    /**
     * Timeout that we set after initially starting the turret PID controller to allow any residual
     * movement from starting the new PID controller to be resolved.
     *
     * Also the delay that we set after onboard mpu6500 is calibrated to ensure that turret IMU has
     * enough time to successfully calibrate.
     */
    tap::arch::MilliTimeout calibrationTimer;

    tap::arch::MilliTimeout buzzerTimer;

    std::vector<int> notes = {1000, 800, 1200, 900, 1000, 0};

    /**
     * Timeout used to determine if we should give up on calibration.
     */
    tap::arch::MilliTimeout calibrationLongTimeout;

    inline bool turretReachedCenterAndNotMoving(
        turret::SentryTurretSubsystem *turret,
        bool ignorePitchTop,
        bool ignorePitchBottom) const
    {
        return (compareFloatClose(
                    0.0f,
                    turret->yawMotorTop.getChassisFrameVelocity(),
                    velocityZeroThreshold) &&
                (turret->yawMotorTop.getChassisFrameMeasuredAngle().minDifference(0) <
                 positionZeroThreshold) &&
                compareFloatClose(
                    0.0f,
                    turret->yawMotorBottom.getChassisFrameVelocity(),
                    velocityZeroThreshold) &&
                (turret->yawMotorBottom.getChassisFrameMeasuredAngle().minDifference(0) <
                 positionZeroThreshold)) &&
               (ignorePitchTop ||
                (compareFloatClose(
                     0.0f,
                     turret->pitchMotorTop.getChassisFrameVelocity(),
                     velocityZeroThreshold) &&
                 (turret->pitchMotorTop.getChassisFrameMeasuredAngle().minDifference(0) <
                  positionZeroThreshold))) &&
               (ignorePitchBottom ||
                (compareFloatClose(
                     0.0f,
                     turret->pitchMotorBottom.getChassisFrameVelocity(),
                     velocityZeroThreshold) &&
                 (turret->pitchMotorBottom.getChassisFrameMeasuredAngle().minDifference(0) <
                  positionZeroThreshold)));
    }
};
}  // namespace src::control::imu

#endif  // IMU_CALIBRATE_COMMAND_HPP_
