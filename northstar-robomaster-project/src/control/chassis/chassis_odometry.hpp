#ifndef CHASSIS_ODOMETRY_HPP
#define CHASSIS_ODOMETRY_HPP

#include "control/chassis/constants/chassis_constants.hpp"
#include "modm/math/geometry/angle.hpp"

namespace src::chassis
{
class ChassisOdometry
{
    static constexpr double ONE_OVER_FOUR_SQRT_TWO = 0.17677669529;

    double RPM_TO_MPS;
    double DIST_TO_CENT;

public:
    // member variables
    double positionGlobal_X;
    double positionGlobal_Y;

    double velocityLocal_X;
    double velocityLocal_Y;

    double rotation;

    uint32_t previousTimeMS = 0;

    ChassisOdometry(float distanceToCenter, float wheelDiameter, float gearRatio)
        : positionGlobal_X(0),
          positionGlobal_Y(0),
          rotation(0),
          RPM_TO_MPS((PI * wheelDiameter / 60.0) / gearRatio),
          DIST_TO_CENT(distanceToCenter)
    {
    }

    double getWorldPositionX() { return positionGlobal_X; }
    double getWorldPositionY() { return positionGlobal_Y; }

    double getLocalVelocityX() { return velocityLocal_X; }
    double getLocalVelocityY() { return velocityLocal_Y; }

    double getRotation() { return rotation; }

    void zeroOdometry()
    {
        positionGlobal_X = 0;
        positionGlobal_Y = 0;
        rotation = 0;
    }

    void updateOdometry(
        int16_t motorRPM_LF,
        int16_t motorRPM_LB,
        int16_t motorRPM_RF,
        int16_t motorRPM_RB)
    {
        uint32_t currentTimeMS = tap::arch::clock::getTimeMilliseconds();
        if (previousTimeMS == 0)
        {
            previousTimeMS = currentTimeMS;
            return;
        }

        double deltaTimeSeconds = (currentTimeMS - previousTimeMS) / 1000.0;
        previousTimeMS = currentTimeMS;

        double velLF = motorRPM_LF * RPM_TO_MPS;
        double velLB = motorRPM_LB * RPM_TO_MPS;
        double velRF = motorRPM_RF * RPM_TO_MPS;
        double velRB = motorRPM_RB * RPM_TO_MPS;

        double localVelX = (velLF + velRF + velLB + velRB) * ONE_OVER_FOUR_SQRT_TWO;
        double localVelY = (velLF - velRF - velLB + velRB) * ONE_OVER_FOUR_SQRT_TWO;

        velocityLocal_X = localVelX;
        velocityLocal_Y = localVelY;

        // may need to flip signs on vars depending on motor direction
        double radiansPerSec = (-velLF - velRF + velLB + velRB) / (4 * DIST_TO_CENT);
        rotation += radiansPerSec * deltaTimeSeconds;

        double cosRot = cos(rotation);
        double sinRot = sin(rotation);
        double globalVelX = cosRot * velocityLocal_X - sinRot * velocityLocal_Y;
        double globalVelY = sinRot * velocityLocal_X + cosRot * velocityLocal_Y;

        positionGlobal_X += globalVelX * deltaTimeSeconds;
        positionGlobal_Y += globalVelY * deltaTimeSeconds;
    }
};

}  // namespace src::chassis

#endif