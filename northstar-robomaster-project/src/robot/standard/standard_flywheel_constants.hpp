#ifndef STANDARD_FLYWHEEL_CONSTANTS_HPP_
#define STANDARD_FLYWHEEL_CONSTANTS_HPP_
#include <modm/container/pair.hpp>

#include "tap/motor/sparkmax/rev_motor.hpp"

#include "modm/math/filter/pid.hpp"

namespace src::control::flywheel
{
static constexpr float FRICTION_WHEEL_RAMP_SPEED = 8.0f;

static constexpr tap::motor::REVMotorId LEFT_MOTOR_ID = tap::motor::REV_MOTOR1;
static constexpr tap::motor::REVMotorId RIGHT_MOTOR_ID = tap::motor::REV_MOTOR3;
static constexpr tap::motor::REVMotorId UP_MOTOR_ID = tap::motor::REV_MOTOR2;

static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS2;

// static constexpr float FLYWHEEL_PID_KP = 10.0f;
// static constexpr float FLYWHEEL_PID_KI = 0.0f;
// static constexpr float FLYWHEEL_PID_KD = 0.0f;
// static constexpr float FLYWHEEL_PID_MAX_ERROR_SUM = 5'000.0f;
// static constexpr float FLYWHEEL_PID_MAX_OUTPUT = 16'000.0f;

// pid constants for spark max flywheels in duty cycle mode
static constexpr float FLYWHEEL_DUTY_PID_KP = 0.0001f;
static constexpr float FLYWHEEL_DUTY_PID_KI = 0.0f;
static constexpr float FLYWHEEL_DUTY_PID_KD = 0.000005f;
static constexpr float FLYWHEEL_DUTY_PID_MAX_ERROR_SUM = 0.05f;
static constexpr float FLYWHEEL_DUTY_PID_MAX_OUTPUT =
    0.15f;  // does not affect feed forward, only the pid output

// TODO make these correct
enum Spin : u_int8_t
{
    SPIN_90 = 0,
    SPIN_100 = 1,
    SPIN_110 = 2,

    SPIN_COUNT
};

static std::array<std::array<modm::Pair<float, float>, 4>, SPIN_COUNT>
    SPIN_TO_INTERPOLATABLE_MPS_TO_DUTY = {
        {{{{0.0f, 0.0f}, {15.0f, .41f}, {18.0f, .50f}, {24.5f, .68f}}},    // SPIN_90
         {{{0.0f, 0.0f}, {15.0f, .41f}, {18.0f, .50f}, {24.5f, .68f}}},    // SPIN_100
         {{{0.0f, 0.0f}, {15.0f, .41f}, {18.0f, .50f}, {24.5f, .68f}}}}};  // SPIN_110

static std::array<std::array<modm::Pair<float, float>, 4>, SPIN_COUNT>
    SPIN_TO_INTERPOLATABLE_MPS_TO_RPM = {
        {{{{0.0f, 0.0f}, {15.0f, 4714.0f}, {18.0f, 5621.0f}, {24.5f, 7700.0f}}},    // SPIN_90
         {{{0.0f, 0.0f}, {15.0f, 4714.0f}, {18.0f, 5621.0f}, {24.5f, 7700.0f}}},    // SPIN_100
         {{{0.0f, 0.0f}, {15.0f, 4714.0f}, {18.0f, 5621.0f}, {24.5f, 7700.0f}}}}};  // SPIN_110

// SPIN_TO_INTERPOLATABLE_MPS_TO_RPM = {
//     {{{{0.0f, 0.0f},
//        {15.0f, 4'500.0f},
//        {18.0f, 5'700.0f},
//        {30.0f, 6'400.0f},
//        {32.0f, 7'000.0f}}},  // SPIN_90
//      {{{0.0f, 0.0f},
//        {15.0f, 4'500.0f},
//        {18.0f, 5'700.0f},
//        {30.0f, 6'400.0f},
//        {32.0f, 7'000.0f}}},  // SPIN_100
//      {{
//          {0.0f, 0.0f},
//          {15.0f, 4'500.0f},
//          {18.0f, 5'700.0f},
//          {30.0f, 6'400.0f},
//          {32.0f, 7'000.0f}  // SPIN_110
//      }}}};

inline std::optional<Spin> toSpinPreset(int value)
{
    switch (value)
    {
        case 90:
            return SPIN_90;
        case 100:
            return SPIN_100;
        case 110:
            return SPIN_110;
        default:
            return SPIN_100;  // invalid input
    }
}

// static std::unordered_map<u_int16_t, std::vector<modm::Pair<float, float>>>
//     SPIN_TO_INTERPOLATABLE_MPS_TO_RPM = {
//         {90, {{0.0f, 0.0f}, {15.0f, .45f}, {18.0f, .57f}, {30.0f, .64f}, {32.0f, .7f}}},
//         {100, {{0.0f, 0.0f}, {15.0f, .45f}, {18.0f, .57f}, {30.0f, .64f}, {32.0f, .7f}}},
//         {110, {{0.0f, 0.0f}, {15.0f, .45f}, {18.0f, .57f}, {30.0f, .64f}, {32.0f, .7f}}}};
}  // namespace src::control::flywheel

#endif