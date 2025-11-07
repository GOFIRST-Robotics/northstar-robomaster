#ifndef CHASSIS_ODOMETRY_HPP
#define CHASSIS_ODOMETRY_HPP

#include "tap/architecture/clock.hpp"

#include "modm/math/geometry/angle.hpp"
#include "modm/math/geometry/vector.hpp"

/*
    Chassis Odometry uses a 2D coordinate system, using the ground as the XY plane
    +X: Right
    +Y: Forward
    +Rotation: CCW
*/

namespace src::chassis
{
class ChassisOdometry
{
    static constexpr float ONE_OVER_THREE = 1.0f / 3.0f;

    tap::communication::sensors::imu::bmi088::Bmi088* imu;
    tap::motor::DjiMotor* turretYaw;

    // rad/sec to m/sec
    float RPS_TO_MPS;
    float DIST_TO_CENT;

    modm::Vector<float, 2> positionGlobal;
    modm::Vector<float, 2> velocityGlobal;
    modm::Vector<float, 2> velocityLocal;
    modm::Vector<float, 3> velocity3dGlobal;
    modm::Vector<float, 2> positionProjectedGlobal;
    modm::Vector<float, 2> velocityProjectedGlobal;

    // radians
    float rotation;

    uint32_t previousTimeMicroSeconds = 0;

public:
    ChassisOdometry(
        tap::communication::sensors::imu::bmi088::Bmi088* imu,
        tap::motor::DjiMotor* turretYaw,
        float distanceToCenter,
        float wheelDiameter)
        : imu(imu),
          turretYaw(turretYaw),
          RPS_TO_MPS(wheelDiameter / 2.0),
          DIST_TO_CENT(distanceToCenter)
    {
        zeroOdometry();
    }

    modm::Vector<float, 2> getPositionGlobal() { return positionGlobal; }
    modm::Vector<float, 2> getVelocityGlobal() { return velocityGlobal; }
    modm::Vector<float, 2> getVelocityLocal() { return velocityLocal; }
    modm::Vector<float, 2> getPositionProjectedGlobal() { return positionProjectedGlobal; }
    modm::Vector<float, 2> getVelocityProjectedGlobal() { return velocityProjectedGlobal; }
    modm::Vector<float, 3> getVelocity3dGlobal() { return velocity3dGlobal; }
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

        velocity3dGlobal = flatLocalVelTo3dGlobalVel(velocityLocal);

        // double radiansPerSec = (mps_LF + mps_RF + mps_LB + mps_RB) / (4 * DIST_TO_CENT);
        // rotation -= radiansPerSec * deltaTimeSeconds;
        rotation = calculateRobotHeading();

        velocityGlobal = convertLocalToGlobal(velocityLocal);
        positionGlobal += velocityGlobal * deltaTimeSeconds;

        velocityProjectedGlobal = modm::Vector<float, 2>(velocity3dGlobal.x, velocity3dGlobal.z);
        positionProjectedGlobal += velocityProjectedGlobal * deltaTimeSeconds;
    }

    modm::Vector<float, 2> convertLocalToGlobal(const modm::Vector<float, 2>& local)
    {
        float cosR = cosf(rotation);
        float sinR = sinf(rotation);

        return modm::Vector<float, 2>(
            local.x * cosR - local.y * sinR,
            local.x * sinR + local.y * cosR);
    }

    float calculateRobotHeading()
    {
        return fmodf(imu->getYaw() + turretYaw->getPositionWrapped(), 2 * M_PI);
    }

    modm::Vector<float, 3> flatLocalVelTo3dGlobalVel(modm::Vector<float, 2> localVel)
    {
        float imuYaw = imu->getYaw();
        float imuRoll = imu->getRoll();
        float imuPitch = imu->getPitch();

        float alpha = calculateRobotHeading();
        float beta = cosf(-imuYaw) * imuRoll + sinf(-imuYaw) * imuPitch;
        float gamma = -sinf(-imuYaw) * imuRoll + cosf(-imuYaw) * imuPitch;

        float x = localVel.x * cosf(beta) * cosf(gamma) +
                  localVel.y * (cosf(alpha) * sinf(beta) * cosf(gamma) + sinf(alpha) * sinf(gamma));
        float y = localVel.x * cosf(beta) * sinf(gamma) +
                  localVel.y * (cosf(alpha) * sinf(beta) * sinf(gamma) - sinf(alpha) * cosf(gamma));
        float z = localVel.x * -sinf(beta) + localVel.y * cosf(alpha) * cosf(beta);

        return modm::Vector<float, 3>(y, x, z);
    }

    float getImuPitch() { return imu->getPitch(); }
    float getImuYaw() { return imu->getYaw(); }
    float getImuRoll() { return imu->getRoll(); }

    // floor length = 2.15m
    // ramp length = 2.80m
};

}  // namespace src::chassis

#endif