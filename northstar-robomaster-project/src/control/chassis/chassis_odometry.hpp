#ifndef CHASSIS_ODOMETRY_HPP
#define CHASSIS_ODOMETRY_HPP

#include "control/chassis/constants/chassis_constants.hpp"
#include "modm/math/geometry/angle.hpp"

namespace src::chassis
{
class ChassisOdometry
{
    static constexpr float ONE_OVER_FOUR_SQRT_TWO = 0.17677669529f;
    static constexpr float SQRT_TWO_OVER_FOUR = 0.35355339059f;

    // rad/sec to m/sec
    float RPS_TO_MPS;
    float DIST_TO_CENT;

public:
    // member variables
    float positionGlobal_X;
    float positionGlobal_Y;

    float velocityLocal_X;
    float velocityLocal_Y;

    float rotation;

    uint32_t previousTimeMS = 0;

    ChassisOdometry(float distanceToCenter, float wheelDiameter)
        : positionGlobal_X(0),
          positionGlobal_Y(0),
          rotation(0),
          RPS_TO_MPS(wheelDiameter / 2.0),
          DIST_TO_CENT(distanceToCenter)
    {
    }

    float getWorldPositionX() { return positionGlobal_X; }
    float getWorldPositionY() { return positionGlobal_Y; }

    float getLocalVelocityX() { return velocityLocal_X; }
    float getLocalVelocityY() { return velocityLocal_Y; }

    float getRotation() { return rotation; }

    void zeroOdometry()
    {
        positionGlobal_X = 0;
        positionGlobal_Y = 0;
        rotation = 0;
    }

    // values are in radians per second
    void updateOdometry(float motorRPS_LF, float motorRPS_LB, float motorRPS_RF, float motorRPS_RB)
    {
        uint32_t currentTimeMS = tap::arch::clock::getTimeMilliseconds();
        if (previousTimeMS == 0)
        {
            previousTimeMS = currentTimeMS;
            return;
        }

        float deltaTimeSeconds = (currentTimeMS - previousTimeMS) / 1000.0f;
        previousTimeMS = currentTimeMS;

        float mps_LF = motorRPS_LF * RPS_TO_MPS;
        float mps_LB = motorRPS_LB * RPS_TO_MPS;
        float mps_RF = motorRPS_RF * RPS_TO_MPS;
        float mps_RB = motorRPS_RB * RPS_TO_MPS;

        // i tihnk this is more real
        float localVelX = (mps_LF + mps_RF - mps_LB - mps_RB) * SQRT_TWO_OVER_FOUR;
        float localVelY = (mps_LF - mps_RF + mps_LB - mps_RB) * SQRT_TWO_OVER_FOUR;

        velocityLocal_X = localVelX;
        velocityLocal_Y = localVelY;

        // float radiansPerSec = (-mps_LF - mps_RF + mps_LB + mps_RB) / (4 * DIST_TO_CENT);
        double radiansPerSec = (mps_LF + mps_RF + mps_LB + mps_RB) / (4 * DIST_TO_CENT);
        rotation += radiansPerSec * deltaTimeSeconds;

        float cosRot = cos(rotation);
        float sinRot = sin(rotation);
        float globalVelX = cosRot * velocityLocal_X - sinRot * velocityLocal_Y;
        float globalVelY = sinRot * velocityLocal_X + cosRot * velocityLocal_Y;

        positionGlobal_X += globalVelX * deltaTimeSeconds;
        positionGlobal_Y += globalVelY * deltaTimeSeconds;
    }
};

}  // namespace src::chassis

#endif