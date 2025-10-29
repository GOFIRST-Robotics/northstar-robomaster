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
          tap::motor::RevMotor::ControlMode::DUTY_CYCLE,
          false,
          "SingleRevMotor",
          18.0f / 120.0f),
      pid(0.0f, 0.0f, 0.0f, 0.8f, 1.0f)
{
}

void RevMotorTesterSingleMotor::initialize() { singularMotor.initialize(); }

float debugVelo;
float debugPos;
float debugCurrent;
float debugPidValue;
void RevMotorTesterSingleMotor::refresh()
{
    // float targetCurrent =
    //     drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL) *
    //     2;
    // pid.update(targetCurrent - singularMotor.getCurrent());
    // debugPidValue = pid.getValue();
    // singularMotor.setControlValue(debugPidValue);
    // debugVelo = singularMotor.getEncoder()->getVelocity() * 60 / M_TWOPI;
    // debugPos = singularMotor.getEncoder()->getPosition().getUnwrappedValue();
    // debugCurrent = singularMotor.getCurrent();
    float targetVoltage =
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
    singularMotor.setControlValue(targetVoltage);
}
}  // namespace Communications::Rev
