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

public:
    ChassisAutoDrive(ChassisSubsystem* chassis, ChassisOdometry* chassisOdometry);

    std::deque<modm::Vector<float, 2>> getPath() { return path; }
    modm::Vector<float, 2> getDesiredGlobalVelocity() { return desiredGlobalVelocity; }

    void resetPath();
    void addPointToPath(modm::Vector<float, 2> newPoint);
    void updateAutoDrive();

private:
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
};

}  // namespace src::chassis

#endif