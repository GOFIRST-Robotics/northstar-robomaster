#ifndef STANDARD_CHASSIS_CONSTANTS_HPP_
#define STANDARD_CHASSIS_CONSTANTS_HPP_

#include "control/chassis/chassis_subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#ifndef CHASSIS_CONSTANTS_HPP_
#error "Do not include this file directly! Use chassis_constants.hpp instead."
#endif

using tap::motor::DjiMotor;
using tap::can::CanBus;

namespace src::chassis
{
    static constexpr float VELOCITY_PID_KP = 1.0f;//10.0f;
    static constexpr float VELOCITY_PID_KI = 0.0f;//0.0f;
    static constexpr float VELOCITY_PID_KD = 0.0f;//1.25f;
    static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 16'000.0f;//0.0f;
    static constexpr float VELOCITY_PID_KV = 0.0f;//0.057f;
    static constexpr float VELOCITY_PID_KS = 0.0f;//350.0f;
    static constexpr float VELOCITY_PID_MAX_OUTPUT = DjiMotor::MAX_OUTPUT_C620;
}

#endif
