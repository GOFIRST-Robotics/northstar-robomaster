#ifndef CHASSIS_ODOMETRY_HPP
#define CHASSIS_ODOMETRY_HPP

#include "control/chassis/constants/chassis_constants.hpp"
#include "modm/math/geometry/angle.hpp"

namespace src::chassis
{
class ChassisOdometry
{
    static constexpr double ONE_OVER_SQRT_TWO = 0.70710678118;

    static constexpr double LOCAL_X_CONTR_LF = ONE_OVER_SQRT_TWO;
    static constexpr double LOCAL_Y_CONTR_LF = ONE_OVER_SQRT_TWO;

    static constexpr double LOCAL_X_CONTR_RF = ONE_OVER_SQRT_TWO;
    static constexpr double LOCAL_Y_CONTR_RF = -ONE_OVER_SQRT_TWO;

    static constexpr double LOCAL_X_CONTR_LB = -ONE_OVER_SQRT_TWO;
    static constexpr double LOCAL_Y_CONTR_LB = ONE_OVER_SQRT_TWO;

    static constexpr double LOCAL_X_CONTR_RB = -ONE_OVER_SQRT_TWO;
    static constexpr double LOCAL_Y_CONTR_RB = -ONE_OVER_SQRT_TWO;

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

    ChassisOdometry(float distanceToCenter, float wheelDiameter)
        : positionGlobal_X(0),
          positionGlobal_Y(0),
          rotation(0),
          RPM_TO_MPS(PI * wheelDiameter / 60.0),
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

        double localVelX = (velLF * LOCAL_X_CONTR_LF + velLB * LOCAL_X_CONTR_LB +
                            velRF * LOCAL_X_CONTR_RF + velRB * LOCAL_X_CONTR_RB) /
                           4.0;
        double localVelY = (velLF * LOCAL_Y_CONTR_LF + velLB * LOCAL_Y_CONTR_LB +
                            velRF * LOCAL_Y_CONTR_RF + velRB * LOCAL_Y_CONTR_RB) /
                           4.0;

        velocityLocal_X = localVelX;
        velocityLocal_Y = localVelY;

        // may need to flip signs on vars depending on motor direction
        double rotContrib = (velLF + velLB + velRF + velRB) / 4.0;
        double radiansPerSec = rotContrib / DIST_TO_CENT;
        rotation += radiansPerSec * deltaTimeSeconds;

        double cosRot = cos(rotation);
        double sinRot = sin(rotation);
        double globalVelX = cosRot * localVelX - sinRot * localVelY;
        double globalVelY = sinRot * localVelX + cosRot * localVelY;

        positionGlobal_X += globalVelX * deltaTimeSeconds;
        positionGlobal_Y += globalVelY * deltaTimeSeconds;
    }
};

}  // namespace src::chassis

#endif