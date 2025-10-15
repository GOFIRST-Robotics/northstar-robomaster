#include "RevMotorTesterSingleMotor.hpp"

#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/motor/sparkmax/rev_motor.hpp"

#include "drivers.hpp"

using tap::algorithms::limitVal;

using namespace tap::motor;
using namespace tap::can;
namespace Communications::Rev
{
RevMotorTesterSingleMotor::RevMotorTesterSingleMotor(tap::Drivers* drivers)
    : Subsystem(drivers),
      singularMotor(
          drivers,
          REVMotorId::REV_MOTOR1,
          tap::can::CanBus::CAN_BUS1,
          tap::motor::RevMotor::ControlMode::VELOCITY,
          false,
          "SingleRevMotor",
          18.0f / 120.0f)
{
}

void RevMotorTesterSingleMotor::initialize()
{
    singularMotor.initialize();
    tap::motor::RevMotor::PIDConfig pidConfig{
        .kP = 0.0f,
        .kI = 0.0f,
        .kD = 0.0f,
        .kF = 0.00009f,
    };
    singularMotor.setMotorPID(pidConfig);
}

float debugVelo;
float debugPos;
void RevMotorTesterSingleMotor::refresh()
{
    singularMotor.setControlValue(
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL) *
        2000);

    debugVelo = singularMotor.getEncoder()->getVelocity() * 60 / M_TWOPI;
    debugPos = singularMotor.getEncoder()->getPosition().getUnwrappedValue();
}
}  // namespace Communications::Rev
