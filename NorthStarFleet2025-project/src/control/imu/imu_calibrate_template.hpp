#ifndef IMU_CALIBRATE_COMMAND_BASE_HPP_
#define IMU_CALIBRATE_COMMAND_BASE_HPP_

#include <vector>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

namespace src::control::imu
{
class ImuCalibrateCommandBase : public tap::control::Command
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

    static constexpr float DEFAULT_VELOCITY_ZERO_THRESHOLD = modm::toRadian(1e-2);
    static constexpr float DEFAULT_POSITION_ZERO_THRESHOLD = modm::toRadian(3.0f);

    const char* getName() const override { return "Calibrate IMU"; }

    virtual bool isReady() override = 0;
    virtual void initialize() override = 0;
    virtual void execute() override = 0;
    virtual void end(bool interrupted) override = 0;
    virtual bool isFinished() const override = 0;

protected:
    static constexpr uint32_t WAIT_TIME_TURRET_RESPONSE_MS = 2000;
    static constexpr uint32_t TURRET_IMU_EXTRA_WAIT_CALIBRATE_MS = 2000;
    static constexpr uint32_t MAX_CALIBRATION_WAITTIME_MS = 20000;

    tap::Drivers* drivers;

    CalibrationState calibrationState = CalibrationState::WAITING_FOR_SYSTEMS_ONLINE;

    uint32_t prevTime = 0;
    tap::arch::MilliTimeout calibrationTimer;
    tap::arch::MilliTimeout buzzerTimer;
    tap::arch::MilliTimeout calibrationLongTimeout;
};

}  // namespace src::control::imu

#endif
