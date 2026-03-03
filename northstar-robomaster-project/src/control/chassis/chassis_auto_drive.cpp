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
        currentT += T_INCREASE;
        return;
    }

    float distanceToEnd = approximateDistanceToEndOfCurve();
    float slowdownMult = 1;

    if (distanceToEnd < SLOWDOWN_DISTANCE)
    {
        slowdownMult = distanceToEnd / SLOWDOWN_DISTANCE;

        if (slowdownMult < 0.15f)
        {
            slowdownMult = 0.15f;
        }
    }

    modm::Vector<float, 2> lookaheadDerivative = getLookaheadDeriv(currentT, T_LOOKAHEAD);
    modm::Vector<float, 2> lookaheadDirection = getDirectionToLookaheadPoint(currentT, T_LOOKAHEAD);
    modm::Vector<float, 2> globalVelocity = chassisOdometry->getVelocityGlobal();

    float dot = lookaheadDirection.normalize().dot(globalVelocity);
    if (dot < 0.5f)
    {
        dot = 0.5f;
    }
    else if (dot > 1.0f)
    {
        dot = 1.0f;
    }

    desiredGlobalVelocity = clampMagnitude(
        ((dirToTarget / distanceToTarget) *
         (lookaheadDerivative.getLength() / lengthOfCurrentCurve())) *
            (slowdownMult * dot),
        MINIMUM_MPS,
        MAXIMUM_MPS);

    calculateRotationToFacePoint(lookaheadDirection);
}

};  // namespace src::chassis