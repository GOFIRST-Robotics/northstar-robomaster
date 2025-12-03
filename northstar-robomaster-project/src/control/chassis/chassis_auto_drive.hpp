#ifndef CHASSIS_AUTO_DRIVE_HPP
#define CHASSIS_AUTO_DRIVE_HPP

#include <deque>

#include "control/algorithms/CubicBezier.hpp"

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

    std::deque<CubicBezier> path;
    float currentT = 0;

    modm::Vector<float, 2> desiredGlobalVelocity;
    float desiredRotation;  // radians per second

public:
    ChassisAutoDrive(ChassisSubsystem* chassis, ChassisOdometry* chassisOdometry);

    std::deque<CubicBezier> getPath() { return path; }
    modm::Vector<float, 2> getDesiredGlobalVelocity() { return desiredGlobalVelocity; }
    float getDesiredRotation() { return desiredRotation; }

    void resetPath();
    void addCurveToPath(CubicBezier newCurve);
    void updateAutoDrive();

    float getOdometryRotation() { return chassisOdometry->getRotation(); }

    modm::Vector<float, 2> getDirectionToCurve(float t)
    {
        return path.front().evaluate(t) - chassisOdometry->getPositionGlobal();
    }

    modm::Vector<float, 2> getLookaheadDeriv(float t, float lookaheadVal)
    {
        float lookahead = t + lookaheadVal;
        if (lookahead > 1)
        {
            lookahead = 1;
        }

        return path.front().evaluateDerivative(lookahead);
    }

private:
    bool tryUpdatePath()
    {
        if (path.size() == 0)
        {
            return false;
        }
        if (currentT > 1)
        {
            path.pop_front();
            currentT = 0;

            if (path.size() == 0)
            {
                return false;
            }
        }

        return true;
    }

    modm::Vector<float, 2> clampMagnitude(modm::Vector<float, 2> orig, float min, float max)
    {
        float length = orig.getLength();

        if (length < min)
        {
            return (orig / length) * min;
        }
        else if (length > max)
        {
            return (orig / length) * max;
        }
        else
        {
            return orig;
        }
    }

    float lengthOfCurrentCurve() { return path.front().getLength(); }
};

}  // namespace src::chassis

#endif