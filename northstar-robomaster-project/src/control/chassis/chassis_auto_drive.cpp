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

void ChassisAutoDrive::resetPath()
{
    path.clear();
    currentT = 0;
}

void ChassisAutoDrive::addCurveToPath(CubicBezier newPoint)
{
    path.push_back(newPoint);

    if (path.size() == 1)
    {
        currentT = approximateTClosestToPoint(chassisOdometry->getPositionGlobal());
    }
}

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

    if (distanceToTarget < 0.0025)
    {
        // currentT += path.front().evaluateDerivative(currentT).getLength() * 0.005f;
        currentT += 0.008f;
        return;
    }

    modm::Vector<float, 2> lookaheadDerivative = getLookaheadDeriv(currentT, 0.05);
    // float desiredFacingRadians = atan2(lookaheadDerivative.x, lookaheadDerivative.y);
    // float d = tap::algorithms::Angle(desiredFacingRadians).minDifference(getOdometryRotation());
    // desiredRotation = d * -2.0f;
    desiredRotation = 0;

    float distanceToEnd = approximateDistanceToEndOfCurve();
    float slowdownMult = 1;

    if (distanceToEnd < 0.5f)
    {
        slowdownMult = distanceToEnd / 0.5f;

        if (slowdownMult < 0.1f)
        {
            slowdownMult = 0.1f;
        }
    }

    desiredGlobalVelocity = clampMagnitude(
        ((dirToTarget / distanceToTarget) * lookaheadDerivative.getLength() /
         lengthOfCurrentCurve()) *
            1.5f * slowdownMult,
        0.4f,
        0.5f);

    xDir = desiredGlobalVelocity.x;
    yDir = desiredGlobalVelocity.y;
}

};  // namespace src::chassis