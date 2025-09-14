#ifndef HERO_AGITATOR_CONSTANTS_HPP_
#define HERO_AGITATOR_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

#include "robot/hero/hero_agitator_config.hpp"

// Do not include this file directly: use agitator_constants.hpp instead.
#ifndef AGITATOR_CONSTANTS_HPP_
#error "Do not include this file directly! Use agitator_constants.hpp instead."
#endif

namespace src::control::agitator::constants
{
static constexpr uint16_t HEAT_LIMIT_BUFFER = 25;

static constexpr tap::algorithms::SmoothPidConfig AGITATOR_PID_CONFIG = {
    .kp = 3500.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 25000.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr src::agitator::HeroAgitatorSubsystemConfig AGITATOR_CONFIG = {
    .agitatorServoId = tap::gpio::Pwm::Pin::C1,
    .shootPwm = 0.56f,
    .reloadPwm = 0.208f,
    .pwmRampSpeed = 0.001f,
    .isAgitatorServoInverted = false,
    .agitatorMotorId = tap::motor::MOTOR4,
    .canBus = tap::can::CanBus::CAN_BUS2,
    .agitatorMotorInverted = true,
    .agitatorGearRatio = 1 / 19.0f,
    .pin = tap::gpio::Digital::InputPin::PF0,
    .limitSwitchInverted = true,
    .reloadTimeout = 10'000};
}  // namespace src::control::agitator::constants

#endif  // HERO_AGITATOR_CONSTANTS_HPP_