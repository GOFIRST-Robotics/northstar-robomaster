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
          false,
          "SingleRevMotor")
{
}

void RevMotorTesterSingleMotor::initialize()
{
    singularMotor.initialize();
    singularMotor.setParameter(Parameter::kP_0, 0.00f);
    singularMotor.setParameter(Parameter::kI_0, 0.00f);
    singularMotor.setParameter(Parameter::kD_0, 0.00f);
    singularMotor.setParameter(Parameter::kF_0, 0.00009f);
    singularMotor.setParameter(Parameter::kOutputMin_0, -1.0f);
    singularMotor.setParameter(Parameter::kOutputMax_0, 1.0f);
    singularMotor.setControlMode(tap::motor::RevMotor::ControlMode::VELOCITY);

    singularMotor.setPeriodicStatusFrame(APICommand::Period0, 0);
    singularMotor.setPeriodicStatusFrame(APICommand::Period1, 2);
    singularMotor.setPeriodicStatusFrame(APICommand::Period2, 2);
    singularMotor.setPeriodicStatusFrame(APICommand::Period3, 0);
    singularMotor.setPeriodicStatusFrame(APICommand::Period4, 0);
}

void RevMotorTesterSingleMotor::refresh()
{
    singularMotor.setControlValue(
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL) *
        2000);
}
}  // namespace Communications::Rev
