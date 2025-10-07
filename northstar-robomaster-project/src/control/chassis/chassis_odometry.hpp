#pragma once

#include "control/chassis/constants/chassis_constants.hpp"
#include "modm/math/geometry/angle.hpp"

namespace src::chassis
{
class ChassisOdometry
{
    // static variables
    static constexpr double LOCAL_X_CONTR_LF = M_SQRT2;
    static constexpr double LOCAL_Y_CONTR_LF = M_SQRT2;

    static constexpr double LOCAL_X_CONTR_RF = -M_SQRT2;
    static constexpr double LOCAL_Y_CONTR_RF = M_SQRT2;

    static constexpr double LOCAL_X_CONTR_LB = -M_SQRT2;
    static constexpr double LOCAL_Y_CONTR_LB = M_SQRT2;

    static constexpr double LOCAL_X_CONTR_RB = M_SQRT2;
    static constexpr double LOCAL_Y_CONTR_RB = M_SQRT2;

    double WHEEL_ROTATIONS_PER_RADIAN;

    // member variables
    double position_X = 0;
    double position_Y = 0;

    double velocity_X = 0;
    double velocity_Y = 0;

    double rotation = 0;

    uint32_t previousTimeMS = 0;

public:
    ChassisOdometry() { WHEEL_ROTATIONS_PER_RADIAN = DIST_TO_CENTER / WHEEL_DIAMETER_M; }

    double getPositionX() { return position_X; }
    double getPositionY() { return position_Y; }

    double getVelocityX() { return velocity_X; }
    double getVelocityY() { return velocity_Y; }

    double getRotation() { return rotation; }

    void zeroOdometry()
    {
        position_X = 0;
        position_Y = 0;
        rotation = 0;
    }

    void updateOdometry(
        int16_t motorRPM_LF,
        int16_t motorRPM_LB,
        int16_t motorRPM_RF,
        int16_t motorRPM_RB)
    {
        uint32_t currentTimeMS = tap::arch::clock::getTimeMilliseconds();
        double deltaTimeSeconds = (currentTimeMS - previousTimeMS) / 1000.0;

        if (previousTimeMS == 0)
        {
            previousTimeMS = currentTimeMS;
            return;
        }

        // Weird units, it's basically RPM contribution (locally) per minute?
        double totalContX = motorRPM_LF * LOCAL_X_CONTR_LF + motorRPM_LB * LOCAL_X_CONTR_LB +
                            motorRPM_RF * LOCAL_X_CONTR_RF + motorRPM_RB * LOCAL_X_CONTR_RB;
        double totalContY = LOCAL_Y_CONTR_LF + motorRPM_LB * LOCAL_Y_CONTR_LB +
                            motorRPM_RF * LOCAL_Y_CONTR_RF + motorRPM_RB * LOCAL_Y_CONTR_RB;

        velocity_X = (totalContX / 60.0) * WHEEL_DIAMETER_M;
        velocity_Y = (totalContY / 60.0) * WHEEL_DIAMETER_M;

        position_X += velocity_X * deltaTimeSeconds;
        position_Y += velocity_Y * deltaTimeSeconds;

        double totalRotationCont =
            motorRPM_LF * WHEEL_ROTATIONS_PER_RADIAN + motorRPM_LB * WHEEL_ROTATIONS_PER_RADIAN +
            motorRPM_RF * WHEEL_ROTATIONS_PER_RADIAN + motorRPM_RB * WHEEL_ROTATIONS_PER_RADIAN;

        double rotationalVelocity = (totalRotationCont / 60.0);
        rotation += rotationalVelocity * deltaTimeSeconds;

        previousTimeMS = currentTimeMS;
    }
};

}  // namespace src::chassis