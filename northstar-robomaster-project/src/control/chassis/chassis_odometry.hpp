#ifndef CHASSIS_ODOMETRY_HPP
#define CHASSIS_ODOMETRY_HPP

#include "tap/architecture/clock.hpp"

#include "modm/math/geometry/angle.hpp"
#include "modm/math/geometry/vector.hpp"


namespace src::chassis
{
class ChassisOdometry
{
    static constexpr float ONE_OVER_THREE = 1.0f / 3.0f;
    static constexpr float THREE_SQRT_TWO_OVER_SIXTEEN = 0.26516504294f;
    static constexpr float ONE_OVER_FOUR_SQRT_TWO = 0.17677669529f;
    static constexpr float SQRT_TWO_OVER_FOUR = 0.35355339059f;

    // rad/sec to m/sec
    float RPS_TO_MPS;
    float DIST_TO_CENT;

    modm::Vector<float, 2> positionGlobal;
    modm::Vector<float, 2> velocityGlobal;
    modm::Vector<float, 2> velocityLocal;

    // radians
    float rotation;

    uint32_t previousTimeMicroSeconds = 0;

public:
    ChassisOdometry(float distanceToCenter, float wheelDiameter)
        : RPS_TO_MPS(wheelDiameter / 2.0),
          DIST_TO_CENT(distanceToCenter)
    {
        zeroOdometry();
    }

    modm::Vector<float, 2> getPositionGlobal() { return positionGlobal; }
    modm::Vector<float, 2> getVelocityGlobal() { return velocityGlobal; }
    modm::Vector<float, 2> getVelocityLocal() { return velocityLocal; }
    float getRotation() { return rotation; }

    void zeroOdometry()
    {
        positionGlobal = modm::Vector<float, 2>(0, 0);
        velocityGlobal = modm::Vector<float, 2>(0, 0);
        velocityLocal = modm::Vector<float, 2>(0, 0);
        rotation = 0;
    }

    // input is in radians per second
    void updateOdometry(float motorRPS_LF, float motorRPS_LB, float motorRPS_RF, float motorRPS_RB)
    {
        uint32_t currentTimeMicroSeconds = tap::arch::clock::getTimeMicroseconds();
        if (previousTimeMicroSeconds == 0)
        {
            previousTimeMicroSeconds = currentTimeMicroSeconds;
            return;
        }

        float deltaTimeSeconds = (currentTimeMicroSeconds - previousTimeMicroSeconds) / 1'000'000.0;
        previousTimeMicroSeconds = currentTimeMicroSeconds;

        float mps_LF = motorRPS_LF * RPS_TO_MPS;
        float mps_LB = motorRPS_LB * RPS_TO_MPS;
        float mps_RF = motorRPS_RF * RPS_TO_MPS;
        float mps_RB = motorRPS_RB * RPS_TO_MPS;

        float localVelX = (mps_LF + mps_RF - mps_LB - mps_RB) * ONE_OVER_THREE;
        float localVelY = (mps_LF - mps_RF + mps_LB - mps_RB) * ONE_OVER_THREE;

        velocityLocal.x = localVelX;
        velocityLocal.y = localVelY;

        double radiansPerSec = (mps_LF + mps_RF + mps_LB + mps_RB) / (4 * DIST_TO_CENT);
        rotation -= radiansPerSec * deltaTimeSeconds;

        velocityGlobal = convertLocalToGlobal(velocityLocal);
        positionGlobal += velocityGlobal * deltaTimeSeconds;
    }

    modm::Vector<float, 2> convertLocalToGlobal(const modm::Vector<float, 2> &local)
    {
        float cosR = cos(rotation);
        float sinR = sin(rotation);

        return modm::Vector<float, 2>(
            local.x * cosR - local.y * sinR,
            local.x * sinR + local.y * cosR);
    }
};

}  // namespace src::chassis

#endif