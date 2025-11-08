#include "chassis_auto_drive.hpp"

#include "tap/algorithms/wrapped_float.hpp"

namespace src::chassis
{
ChassisAutoDrive::ChassisAutoDrive(
    ChassisSubsystem* chassis,
    src::chassis::ChassisOdometry* chassisOdometry)
    : chassis(chassis),
      chassisOdometry(chassisOdometry),
      path(std::deque<CubicBezier>())

{
}

void ChassisAutoDrive::resetPath() { path.clear(); }

void ChassisAutoDrive::addCurveToPath(CubicBezier newPoint) { path.push_back(newPoint); }

float xDir;
float yDir;

void ChassisAutoDrive::updateAutoDrive()
{
    if (!tryUpdatePath())
    {
        desiredGlobalVelocity = modm::Vector<float, 2>(0, 0);
        desiredRotation = 0;
        return;
    }

    modm::Vector<float, 2> dirToTarget = getDirectionToCurve(currentT);
    float distanceToTarget = dirToTarget.getLength();

    if (distanceToTarget < 0.05)
    {
        currentT += path.front().evaluateDerivative(currentT).getLength() * 0.01f;
        return;
    }

    modm::Vector<float, 2> lookaheadDerivative = getLookaheadDeriv(currentT, 0.05);
    float desiredFacingRadians = atan2(lookaheadDerivative.x, lookaheadDerivative.y);
    float d = tap::algorithms::Angle(desiredFacingRadians).minDifference(getOdometryRotation());
    desiredRotation = d * -1.2;
    desiredGlobalVelocity = (dirToTarget / distanceToTarget) * 0.5f;

    xDir = desiredGlobalVelocity.x;
    yDir = desiredGlobalVelocity.y;
}

};  // namespace src::chassis