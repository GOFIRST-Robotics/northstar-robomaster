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

    if (distanceToTarget < T_CHECK)
    {
        // currentT += path.front().evaluateDerivative(currentT).getLength() * 0.005f;
        currentT += T_INCREASE;
        return;
    }

    float distanceToEnd = approximateDistanceToEndOfCurve();
    float slowdownMult = 1;

    if (distanceToEnd < SLOWDOWN_DISTANCE)
    {
        slowdownMult = distanceToEnd / SLOWDOWN_DISTANCE;

        if (slowdownMult < 0.1f)
        {
            slowdownMult = 0.1f;
        }
    }

    modm::Vector<float, 2> lookaheadDerivative = getLookaheadDeriv(currentT, T_LOOKAHEAD);
    modm::Vector<float, 2> lookaheadDirection = getDirectionToLookaheadPoint(currentT, T_LOOKAHEAD);

    desiredGlobalVelocity = clampMagnitude(
        ((dirToTarget / distanceToTarget) * lookaheadDerivative.getLength() /
         lengthOfCurrentCurve()) *
            1.5f * slowdownMult,
        MINIMUM_MPS,
        MAXIMUM_MPS);

    xDir = desiredGlobalVelocity.x;
    yDir = desiredGlobalVelocity.y;

    calculateRotationToFacePoint(lookaheadDirection, desiredGlobalVelocity.getLength());
}

};  // namespace src::chassis