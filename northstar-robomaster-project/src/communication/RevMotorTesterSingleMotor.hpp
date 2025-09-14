#pragma once

#include <array>

#include "tap/control/subsystem.hpp"
#include "tap/util_macros.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/sparkmax/rev_motor.hpp"
#endif

// class Drivers;

namespace Communications::Rev
{
///
/// @brief This subsystem encapsulates four motors that control the chassis.
///
class RevMotorTesterSingleMotor : public tap::control::Subsystem
{
public:
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    using Motor = testing::NiceMock<tap::mock::DjiMotorMock>;
#else
    using Motor = tap::motor::RevMotor;
#endif

    RevMotorTesterSingleMotor(tap::Drivers* drivers);

    ///
    /// @brief Initializes the drive motors.
    ///
    void initialize() override;

    ///
    /// @brief Runs velocity PID controllers for the drive motors.
    ///
    void refresh() override;

    const char* getName() { return "RevMotorTesterSubsytem"; }

private:
    Motor singularMotor;

};  // class ChassisSubsystem
}  // namespace Communications::Rev
