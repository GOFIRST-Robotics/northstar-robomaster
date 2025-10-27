#ifndef STANDARD_CHASSIS_CONSTANTS_HPP_
#define STANDARD_CHASSIS_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"

#include "control/chassis/chassis_subsystem.hpp"

#ifndef CHASSIS_CONSTANTS_HPP_
#error "Do not include this file directly! Use chassis_constants.hpp instead."
#endif

using tap::can::CanBus;
using tap::motor::DjiMotor;

namespace src::chassis
{
static constexpr float VELOCITY_PID_KP = 15.0f;                 // 10.0f;
static constexpr float VELOCITY_PID_KI = 0.0f;                  // 0.0f;
static constexpr float VELOCITY_PID_KD = 1.0f;                  // 1.25f;
static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 16'000.0f;  // 0.0f;
static constexpr float VELOCITY_PID_KV = 0.0f;                  // 0.057f;
static constexpr float VELOCITY_PID_KS = 0.0f;                  // 350.0f;
static constexpr float VELOCITY_PID_MAX_OUTPUT = DjiMotor::MAX_OUTPUT_C620;
static constexpr float CHASSIS_ROTATION_P = 0.9f;
static constexpr float CHASSIS_ROTATION_D = 0.01f;
static constexpr float CHASSIS_ROTATION_MAX_VEL = 1.0f;
static constexpr float AUTO_ROTATION_ALPHA = 0.01f;

static constexpr float CHASSIS_GEAR_RATIO = (187.0f / 3591.0f);

static const float DIST_TO_CENTER = .34f;  // from wheel to center
static const float WHEEL_DIAMETER_M = 0.120f;
static const float RAMP_UP_RPM_INCREMENT_MPS = 0.01f;
}  // namespace src::chassis

#endif
