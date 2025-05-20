/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef PLATE_HIT_GOVERNOR_HPP_
#define PLATE_HIT_GOVERNOR_HPP_

#include <cassert>

#include "tap/architecture/clock.hpp"
#include "tap/control/governor/command_governor_interface.hpp"
#include "tap/drivers.hpp"

#include "ref_system_constants.hpp"

namespace src::control::governor
{
/**
 * Governor that blocks commands from running if a plate has been hit recently.
 */
class PlateHitGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    /**
     * @param durationBuffer Time since last hit in milliseconds to run the command blocked.
     */
    PlateHitGovernor(tap::Drivers* drivers, const uint32_t durationBuffer)
        : drivers(drivers),
          durationBuffer(durationBuffer)
    {
    }

    bool isReady() final { return enoughTimeSinceLastHit(); }

    bool isFinished() final { return !enoughTimeSinceLastHit(); }

private:
    tap::Drivers* drivers;

    const uint32_t durationBuffer;

    float lastDPS;
    float currentDPS;
    uint32_t timeSinceLastHit;

    bool enoughTimeSinceLastHit()
    {
        if (drivers->refSerial.getRobotData().receivedDps > lastDPS)
        {
            timeSinceLastHit = tap::arch::clock::getTimeMilliseconds();
        }
        lastDPS = drivers->refSerial.getRobotData().receivedDps;
        const uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();

        return currentTime - timeSinceLastHit > durationBuffer;
    }
};
}  // namespace src::control::governor

#endif  //  PLATE_HIT_GOVERNOR_HPP_
