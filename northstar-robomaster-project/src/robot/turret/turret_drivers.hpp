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

#ifndef TURRET_DRIVERS_HPP_
#define TURRET_DRIVERS_HPP_
 
#include "tap/drivers.hpp"
 
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/imu_terminal_serial_handler_mock.hpp"
#else
#include "tap/communication/sensors/imu/imu_terminal_serial_handler.hpp"
#include "../../src/communication/can/chassis/chassis_mcb_can_comm.hpp"
#endif

namespace src::gyro
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers()
        : tap::Drivers(),
        chassisMcbCanComm(this)
    {
    }
 
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)

#else
public:
    ChassisMcbCanComm chassisMcbCanComm;
#endif
};  // class src::TurretDrivers
}  // namespace src::turret

#endif  // TURRET_DRIVERS_HPP_
 