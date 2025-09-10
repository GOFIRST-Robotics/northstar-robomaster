
#ifndef WORLD_FRAME_PITCH_CHASSIS_IMU_COMP_TURRET_CONTROLLER_HPP_
#define WORLD_FRAME_PITCH_CHASSIS_IMU_COMP_TURRET_CONTROLLER_HPP_

#include <cstdint>

#include "tap/algorithms/fuzzy_pd.hpp"
#include "tap/algorithms/wrapped_float.hpp"
#include "tap/drivers.hpp"

#include "control/turret/turret_motor.hpp"
#include "robot/sentry/sentry_turret_subsystem.hpp"

using namespace tap::algorithms;

namespace src::control::turret
{
class TurretMotor;
}

namespace src::control::turret::algorithms
{
class WorldFramePitchChassisImuCompTurretController final : public TurretPitchControllerInterface
{
public:
    /**
     * @param[in] drivers A drivers object that will be queried for IMU information.
     * @param[in] pitchMotor A `TurretMotor` object accessible for children objects to use.
     * @param[in] pidConfig PID configuration struct for the controller.
     */
    WorldFramePitchChassisImuCompTurretController(
        tap::Drivers &drivers,
        TurretMotor &pitchMotor,
        const tap::algorithms::SmoothPidConfig &pidConfig,
        src::control::turret::SentryTurretSubsystem *turretSubsystem);

    void initialize() final;

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] desiredSetpoint The pitch desired setpoint in the world frame.
     */
    void runController(const uint32_t dt, const WrappedFloat desiredSetpoint) final;

    /// @return World frame pitch angle setpoint, refer to top level documentation for more details.
    void setSetpoint(WrappedFloat desiredSetpoint) final;

    /// @return world frame pitch angle measurement, refer to top level documentation for more
    /// details.
    WrappedFloat getMeasurement() const final;

    /**
     * @return The pitch setpoint, in the world frame.
     */
    inline WrappedFloat getSetpoint() const final { return worldFrameSetpoint; }

    bool isOnline() const final;

    WrappedFloat convertControllerAngleToChassisFrame(
        WrappedFloat controllerFrameAngle) const final;

    WrappedFloat convertChassisAngleToControllerFrame(WrappedFloat chassisFrameAngle) const final;

private:
    tap::Drivers &drivers;

    tap::algorithms::SmoothPid pid;

    WrappedFloat worldFrameSetpoint;

    WrappedFloat chassisFrameInitImuPitchAngle;

    src::control::turret::SentryTurretSubsystem *turretSubsystem;

    inline WrappedFloat getBmi088Pitch() const
    {
        return Angle(
            drivers.bmi088.getRoll() * cosf(-turretSubsystem->yawMotorTop.getTurretController()
                                                 ->getMeasurement()
                                                 .getUnwrappedValue()) +
            drivers.bmi088.getPitch() * sinf(-turretSubsystem->yawMotorTop.getTurretController()
                                                  ->getMeasurement()
                                                  .getUnwrappedValue()));
    }

    inline float getPitchVelo() const
    {
        return drivers.bmi088.getGx() * cosf(-turretSubsystem->yawMotorTop.getTurretController()
                                                  ->getMeasurement()
                                                  .getUnwrappedValue()) +
               drivers.bmi088.getGy() * sinf(-turretSubsystem->yawMotorTop.getTurretController()
                                                  ->getMeasurement()
                                                  .getUnwrappedValue());
    }
};

}  // namespace src::control::turret::algorithms

#endif  // WORLD_FRAME_PITCH_CHASSIS_IMU_COMP_TURRET_CONTROLLER_HPP_
