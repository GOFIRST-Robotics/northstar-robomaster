#include "RevMotorTesterSingleMotor.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/motor/sparkmax/rev_motor.hpp"

#include "drivers.hpp"

//#define THING

#include <cmath>

using tap::algorithms::limitVal;

namespace Communications::Rev
{
// STEP 1 (Tank Drive): create constructor
RevMotorTesterSingleMotor::RevMotorTesterSingleMotor(tap::Drivers* drivers)
    : Subsystem(drivers),
      singularMotor(
          drivers,
          tap::motor::REVMotorId::REV_MOTOR1,
          tap::can::CanBus::CAN_BUS1,
          false,
          "SingleRevMotor")
{
}

// STEP 2 (Tank Drive): initialize function
void RevMotorTesterSingleMotor::initialize()
{
    singularMotor.initialize();
    singularMotor.setParameter(tap::motor::Parameter::kP_0, 0.00f);
    singularMotor.setParameter(tap::motor::Parameter::kI_0, 0.00f);
    singularMotor.setParameter(tap::motor::Parameter::kD_0, 0.00f);
    singularMotor.setParameter(tap::motor::Parameter::kF_0, 0.05f);
    singularMotor.setParameter(tap::motor::Parameter::kOutputMin_0, -0.1f);
    singularMotor.setParameter(tap::motor::Parameter::kOutputMax_0, 0.1f);
    // singularMotor.setParameter(tap::motor::Parameter::, 0.1f);
    singularMotor.setControlMode(
        tap::motor::RevMotor::ControlMode::CURRENT);  // Set control mode to DUTY_CYCLE
    // singularMotor.setControlValue(1.0f);              // Initialize control value to 0.0f

    // motor1.setTargetVoltage(0.1f); //causing error bacause setTargetVoltage dosen't exist
}

// void RevMotorTester::initialize()
// {
//     motor1.initialize();
//     motor2.initialize();
//     motor3.initialize();
//     // motor1.setTargetVoltage(0.1f); //causing error bacause setTargetVoltage dosen't exist
//     motor1.setControlValue(0.25f);
//     motor2.setControlValue(0.25f);
//     motor3.setControlValue(0.25f);
// } w

void RevMotorTesterSingleMotor::refresh()
{
    singularMotor.setControlValue(
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL));
    // motor1.setTargetVoltage(0.1f);
}
// STEP
}  // namespace Communications::Rev
