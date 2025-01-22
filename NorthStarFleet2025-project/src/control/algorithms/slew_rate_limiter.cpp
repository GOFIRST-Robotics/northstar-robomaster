#include "slew_rate_limiter.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "tap/architecture/clock.hpp"

#include <iostream>

using tap::arch::clock::getTimeMilliseconds;

namespace control::algorithms
{

float SlewRateLimiter::runLimiter(float desiredVelocity, float currentVelocity) {
    timeDiff = getTimeMilliseconds() - prevTime;
    prevTime = getTimeMilliseconds(); // Might not work

    if (abs(desiredVelocity - (currentVelocity + timeDiff * rateLimit)) > maxError) {
        if (desiredVelocity < currentVelocity) {
            return currentVelocity + timeDiff * -rateLimit;
        } else {
            return currentVelocity + timeDiff * rateLimit;
        }
    } else {
        return desiredVelocity;
    }

}

}