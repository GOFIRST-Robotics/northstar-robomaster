#ifndef IMU_CALIBRATING_HPP
#define IMU_CALIBRATING_HPP

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/governor/command_governor_interface.hpp"
#include "tap/drivers.hpp"

#include "control/imu/imu_calibrate_command.hpp"

namespace src::control::governor
{
class ImuCalibratingGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    ImuCalibratingGovernor(
        tap::Drivers *drivers,
        src::control::imu::ImuCalibrateCommand &imuCommand)
        : drivers(drivers),
          imuCommand(imuCommand)
    {
    }

    bool isReady() final { return !drivers->commandScheduler.isCommandScheduled(&imuCommand); }

    bool isFinished() final { return !isReady(); }

private:
    tap::Drivers *drivers;
    src::control::imu::ImuCalibrateCommand &imuCommand;
};
}  // namespace src::control::governor

#endif  // IMU_CALIBRATING_HPP
