#ifndef STANDARD_CHASSIS_CONSTANTS_HPP_
#define STANDARD_CHASSIS_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"

#include "modm/math/interpolation/linear.hpp"

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

static constexpr float MAX_CHASSIS_SPEED_MPS = 4.0f;

static constexpr modm::Pair<int, float> CHASSIS_POWER_TO_MAX_SPEED_LUT[] = {
    {50, 4'500},
    {60, 5'700},
    {70, 6'400},
    {80, 6'700},
    {100, 7'000},
    {120, 8'000},
};

static constexpr modm::Pair<int, float> CHASSIS_POWER_TO_MAX_ACCEL_LUT[] = {
    {50, 0.006},
    {60, 0.008},
    {70, 0.01},
    {80, 0.012},
    {100, 0.014},
    {120, 0.016},
};

static constexpr float CHASSIS_DECCEL_VALUE = 0.04f;

static modm::interpolation::Linear<modm::Pair<int, float>> CHASSIS_POWER_TO_SPEED_INTERPOLATOR(
    CHASSIS_POWER_TO_MAX_SPEED_LUT,
    MODM_ARRAY_SIZE(CHASSIS_POWER_TO_MAX_SPEED_LUT));

static modm::interpolation::Linear<modm::Pair<int, float>> CHASSIS_POWER_TO_ACCEL_INTERPOLATOR(
    CHASSIS_POWER_TO_MAX_ACCEL_LUT,
    MODM_ARRAY_SIZE(CHASSIS_POWER_TO_MAX_ACCEL_LUT));

}  // namespace src::chassis

#endif
