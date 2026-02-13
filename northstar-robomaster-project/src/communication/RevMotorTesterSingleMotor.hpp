#pragma once

#include <array>

#include "tap/control/subsystem.hpp"
#include "tap/util_macros.hpp"

#include "modm/math/filter/pid.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/sparkmax/rev_motor.hpp"
#endif

namespace Communications::Rev
{
class RevMotorTesterSingleMotor : public tap::control::Subsystem
{
public:
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    using Motor = testing::NiceMock<tap::mock::DjiMotorMock>;
#else
    using Motor = tap::motor::RevMotor;
#endif

    RevMotorTesterSingleMotor(tap::Drivers* drivers);

    void initialize() override;

    void refresh() override;

    const char* getName() { return "RevMotorTesterSubsytem"; }

private:
    modm::Pid<float> pid;

    Motor singularMotor;

};  // class ChassisSubsystem
}  // namespace Communications::Rev
