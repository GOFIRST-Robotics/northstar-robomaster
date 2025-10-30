#ifndef CHASSIS_AUTO_DRIVE_HPP
#define CHASSIS_AUTO_DRIVE_HPP

#include <deque>

#include "chassis_odometry.hpp"
#include "chassis_subsystem.hpp"

namespace src::chassis
{
class ChassisAutoDrive
{
    static constexpr float MAXIMUM_MPS = 1.0f;
    static constexpr float MINIMUM_MPS = 0.38f;

    static constexpr float MAX_POSITION_ERROR = 0.02f;

    src::chassis::ChassisSubsystem* chassis;
    src::chassis::ChassisOdometry* chassisOdometry;

    std::deque<modm::Vector<float, 2>> path;

    modm::Vector<float, 2> desiredGlobalVelocity;
    float desiredRotation;  // radians per second

public:
    ChassisAutoDrive(ChassisSubsystem* chassis, ChassisOdometry* chassisOdometry);

    std::deque<modm::Vector<float, 2>> getPath() { return path; }
    modm::Vector<float, 2> getDesiredGlobalVelocity() { return desiredGlobalVelocity; }
    float getDesiredRotation() { return desiredRotation; }

    void resetPath();
    void addPointToPath(modm::Vector<float, 2> newPoint);
    void updateAutoDrive();

    float getOdometryRotation() { return chassisOdometry->getRotation(); }

    modm::Vector<float, 2> getDirectionToCurve(float t)
    {
        return evaluateCurve(t) - chassisOdometry->getPositionGlobal();
    }

    modm::Vector<float, 2> getLookaheadDeriv(float t, float lookaheadVal)
    {
        float lookahead = t + lookaheadVal;
        if (lookahead > 1)
        {
            lookahead = 1;
        }

        return evaluateDerivCurve(lookahead);
    }

private:
    modm::Vector<float, 2> currentIdealVelocity;

    bool tryUpdatePath()
    {
        if (path.size() == 0)
        {
            return false;
        }
        if (chassisOdometry->getPositionGlobal().getDistanceTo(path[0]) <= MAX_POSITION_ERROR)
        {
            path.pop_front();
            if (path.size() == 0)
            {
                return false;
            }
        }

        return true;
    }

    // Quadratic Bézier Curve
    // P(t) = (1-t)² * A + 2*(1-t)*t * C + t² * B
    modm::Vector<float, 2> evaluateCurve(float t)
    {
        modm::Vector<float, 2> A = modm::Vector<float, 2>(0, 0);
        modm::Vector<float, 2> B = modm::Vector<float, 2>(2, 1.2);
        modm::Vector<float, 2> C = modm::Vector<float, 2>(-1.2, 1);

        float oneMinusT = 1 - t;
        return oneMinusT * oneMinusT * A + 2 * oneMinusT * t * C + (t * t) * B;
    }

    // P'(t) = 2*[(1-t)*(C - A) + t*(B - C)]
    modm::Vector<float, 2> evaluateDerivCurve(float t)
    {
        modm::Vector<float, 2> A = modm::Vector<float, 2>(0, 0);
        modm::Vector<float, 2> B = modm::Vector<float, 2>(2, 1.2);
        modm::Vector<float, 2> C = modm::Vector<float, 2>(-1.2, 1);

        return 2 * ((1 - t) * (C - A) + t * (B - C));
    }
};

}  // namespace src::chassis

#endif