#include "slew_rate_limiter.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace control::algorithms
{

float SlewRateLimiter::runLimiter(float desiredVelocity, float currentVelocity) {
    if (abs(desiredVelocity - (currentVelocity + rateLimit)) > maxError) {
        if (desiredVelocity < currentVelocity) {
            return currentVelocity - rateLimit;
        }/* else {
            return currentVelocity + rateLimit;
        }*/
    } else {
        return desiredVelocity;
    }

}

}