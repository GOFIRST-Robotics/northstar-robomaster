#ifndef CHASSIS_CONSTANTS_HPP_
#define CHASSIS_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"

#ifdef TARGET_STANDARD
#include "robot/standard/standard_chassis_constants.hpp"
#elif TARGET_SENTRY
#include "robot/sentry/sentry_chassis_constants.hpp"
#elif TARGET_HERO
#include "robot/hero/hero_chassis_constants.hpp"
#elif TURRET
#include "robot/standard/standard_chassis_constants.hpp"
#elif TARGET_TEST_BED
#include "robot/hero/hero_chassis_constants.hpp"
#else
#include "robot/standard/standard_chassis_constants.hpp"
#endif

namespace src::chassis
{
// hardware constants, not specific to any particular chassis
static constexpr tap::motor::MotorId LEFT_FRONT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId LEFT_BACK_MOTOR_ID = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId RIGHT_FRONT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId RIGHT_BACK_MOTOR_ID = tap::motor::MOTOR4;

static modm::Pair<float, float> getNormalizedInput(float vert, float hor)
{
    float dist = sqrt((vert * vert) + (hor * hor));
    if (dist > 1.0f)
    {
        return modm::Pair<float, float>(vert / dist, hor / dist);
    }
    else
    {
        return modm::Pair<float, float>(vert, hor);
    }
}

}  // namespace src::chassis

#endif  // CHASSIS_CONSTANTS_HPP_
