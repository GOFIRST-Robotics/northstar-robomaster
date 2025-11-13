#ifndef CUBIC_BEZIER_HPP
#define CUBIC_BEZIER_HPP

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "control/chassis/chassis_auto_drive.hpp"
#include "control/chassis/chassis_subsystem.hpp"

class CubicBezier
{
public:
    CubicBezier(
        modm::Vector<float, 2> start,
        modm::Vector<float, 2> end,
        modm::Vector<float, 2> startControl,
        modm::Vector<float, 2> endControl)
        : start(start),
          end(end),
          startControl(startControl),
          endControl(endControl)
    {
        length = estimateLength();
    }

    CubicBezier(
        modm::Vector<float, 2> start,
        modm::Vector<float, 2> end,
        modm::Vector<float, 2> startControl,
        modm::Vector<float, 2> endControl,
        float length)
        : start(start),
          end(end),
          startControl(startControl),
          endControl(endControl),
          length(length)
    {
    }

    modm::Vector<float, 2> getStart() { return start; }
    modm::Vector<float, 2> getEnd() { return end; }
    modm::Vector<float, 2> getStartControl() { return startControl; }
    modm::Vector<float, 2> getEndControl() { return endControl; }
    float getLength() { return length; }

    modm::Vector<float, 2> evaluate(float t)
    {
        float oneMinusT = 1 - t;
        return (oneMinusT * oneMinusT * oneMinusT) * start +
               (3.0f * (oneMinusT * oneMinusT) * t) * startControl +
               (3.0f * oneMinusT * (t * t)) * endControl + (t * t * t) * end;
    }

    modm::Vector<float, 2> evaluateDerivative(float t)
    {
        float oneMinusT = 1 - t;
        return (3.0f * (oneMinusT * oneMinusT)) * (startControl - start) +
               (6.0f * oneMinusT * t) * (endControl - startControl) +
               (3.0f * (t * t)) * (end - endControl);
    }

    modm::Vector<float, 2> evaluateSecondDerivative(float t)
    {
        return (6 * (1 - t)) * (endControl - 2.0f * startControl + start) +
               (6 * t) * (end - 2 * endControl + startControl);
    }

    float estimateLength()
    {
        float length = 0.0f;
        modm::Vector<float, 2> prevPoint = start;

        for (int i = 0; i < 16; i++)
        {
            modm::Vector<float, 2> point = evaluate((i + 1) / 16.0f);
            length += prevPoint.getDistanceTo(point);
            prevPoint = point;
        }

        return length;
    }

private:
    modm::Vector<float, 2> start;
    modm::Vector<float, 2> end;
    modm::Vector<float, 2> startControl;
    modm::Vector<float, 2> endControl;

    float length;
};

#endif