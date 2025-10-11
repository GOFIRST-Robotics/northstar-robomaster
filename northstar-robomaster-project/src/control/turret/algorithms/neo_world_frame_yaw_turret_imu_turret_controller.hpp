#ifndef NEO_WORLD_FRAME_YAW_TURRET_IMU_TURRET_CONTROLLER_HPP_
#define NEO_WORLD_FRAME_YAW_TURRET_IMU_TURRET_CONTROLLER_HPP_

#include <cstdint>

#include "tap/algorithms/fuzzy_pd.hpp"
#include "tap/algorithms/wrapped_float.hpp"
#include "tap/drivers.hpp"

#include "control/turret/turret_motor.hpp"

#include "turret_controller_interface.hpp"

using namespace tap::algorithms;

namespace src::control::turret::algorithms
{
/**
 * World frame turret yaw controller. Requires that a development board be mounted rigidly on the
 * turret. The development board's IMU is used to determine the turret's world frame coordinates
 * directly, making this controller better than the `WorldFrameChassisImuTurretController`.
 *
 * Runs a cascade PID controller (position PID output feeds into velocity PID controller, velocity
 * PID controller is desired motor output) to control the turret yaw.
 *
 * Implements TurretYawControllerInterface interface, see parent class comment for details.
 */
class NeoWorldFrameYawTurretImuCascadePidTurretController : public TurretYawControllerInterface
{
public:
    /**
     * @param[in] drivers A drivers object that will be queried for IMU information.
     * @param[in] yawMotor A `TurretMotor` object accessible for children objects to use.
     * @param[in] positionPid Position PID controller.
     * @param[in] velocityPid Velocity PID controller.
     */
    NeoWorldFrameYawTurretImuCascadePidTurretController(
        tap::Drivers &drivers,
        TurretMotor &yawMotor,
        SmoothPid &positionPid,
        SmoothPid &velocityPid);

    void initialize();

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] desiredSetpoint The unwrapped yaw desired setpoint in the world frame. Clamped
     * within chassis frame turret angle limits if applicable.
     */
    void runController(const uint32_t dt, const WrappedFloat desiredSetpoint);

    /// Sets the world frame yaw angle setpoint, refer to top level documentation for more details.
    void setSetpoint(WrappedFloat desiredSetpoint);

    /// @return World frame yaw angle setpoint, refer to top level documentation for more details.
    inline WrappedFloat getSetpoint() const { return worldFrameSetpoint; }

    /// @return World frame yaw angle measurement from IMU, refer to top level documentation for
    /// more details.
    WrappedFloat getMeasurement() const;

    /// @return World frame yaw angle measurement from MOTOR, refer to top level documentation for
    /// more details.
    WrappedFloat getMeasurementMotor() const;

    bool isOnline() const;

    WrappedFloat convertControllerAngleToChassisFrame(WrappedFloat controllerFrameAngle) const;

    WrappedFloat convertChassisAngleToControllerFrame(WrappedFloat chassisFrameAngle) const;

private:
    tap::Drivers &drivers;

    SmoothPid &positionPid;
    SmoothPid &velocityPid;

    WrappedFloat worldFrameSetpoint;

    float worldFrameMeasurementIMU;
    int32_t IMUrevolutions;

    inline WrappedFloat getBmi088Yaw(bool negitive = false) const
    {
        return negitive ? Angle(drivers.bmi088.getYaw() * -1) : Angle(drivers.bmi088.getYaw());
    }

    inline float getBmi088YawVelocity() const { return drivers.bmi088.getGz(); }
};

}  // namespace src::control::turret::algorithms

#endif  //  NEO_WORLD_FRAME_YAW_TURRET_IMU_TURRET_CONTROLLER_HPP_
