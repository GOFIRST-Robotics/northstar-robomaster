#include "RevMotorTesterSingleMotor.hpp"

#include "tap/algorithms/math_user_utils.hpp"

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
          tap::motor::REVMotorId::REV_MOTOR6,
          tap::can::CanBus::CAN_BUS2,
          false,
          "SingleRevMotor")
{
}

// STEP 2 (Tank Drive): initialize function
    void RevMotorTesterSingleMotor::initialize() {
        singularMotor.initialize();
        singularMotor.setControlMode(tap::motor::RevMotor::ControlMode::VOLTAGE);
        singularMotor.setControlValue(0.05f); // Initialize control value to 0.0f
        
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
// }

void RevMotorTesterSingleMotor::refresh()
{
    // motor1.setTargetVoltage(0.1f);
}
// STEP
}  // namespace Communications::Rev
