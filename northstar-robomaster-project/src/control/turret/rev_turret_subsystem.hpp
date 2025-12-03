#ifndef REV_TURRET_SUBSYSTEM_HPP_
#define REV_TURRET_SUBSYSTEM_HPP_

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/algorithms/wrapped_float.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/control/turret_subsystem_interface.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/sparkmax/rev_motor.hpp"

#include "turret_motor_config.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "src/mock/turret_motor_mock.hpp"
#else
#include "turret_double_motor_rev.hpp"
#include "turret_motor_GM6020.hpp"

#endif

#include "tap/util_macros.hpp"

#include "modm/math/filter/pid.hpp"

namespace src::control::turret
{
/**
 * Stores software necessary for interacting with two gimbals that control the pitch and
 * yaw of a turret. Provides a convenient API for other commands to interact with a turret.
 *
 * All angles computed using a right hand coordinate system. In other words, yaw is a value from
 * 0-M_TWOPI rotated counterclockwise when looking at the turret from above. Pitch is a value from
 * 0-M_TWOPI rotated counterclockwise when looking at the turret from the right side of the turret.
 */
class RevTurretSubsystem : public tap::control::Subsystem
{
public:
    /**
     * Constructs a RevTurretSubsystem.
     *
     * @param[in] pitchMotor Pointer to pitch motor that this `RevTurretSubsystem` will own.
     * @param[in] yawMotor1 Pointer to yaw motor that this `RevTurretSubsystem` will own.
     * @param[in] yawMotor2 Pointer to yaw motor that this `RevTurretSubsystem` will own.
     */
    explicit RevTurretSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface* pitchMotor,
        tap::motor::RevMotor* yawMotor1,
        tap::motor::RevMotor* yawMotor2,
        const TurretMotorConfig& pitchMotorConfig,
        const TurretMotorConfig& yawMotorConfig);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        yawMotor.setMotorOutput(0);
        pitchMotor.setMotorOutput(0);
    }

    const char* getName() const override { return "Turret"; }

    mockable inline bool isOnline() const { return pitchMotor.isOnline() && yawMotor.isOnline(); }

#ifdef ENV_UNIT_TESTS
    testing::NiceMock<mock::TurretMotorMock> pitchMotor;
    testing::NiceMock<mock::TurretMotorMock> yawMotor;
#else
    /// Associated with and contains logic for controlling the turret's pitch motor
    TurretMotorGM6020 pitchMotor;
    /// Associated with and contains logic for controlling the turret's yaw motor
    TurretDoubleMotorRev yawMotor;
#endif

};  // class RevTurretSubsystem

}  // namespace src::control::turret

#endif  // REV_TURRET_SUBSYSTEM_HPP_
