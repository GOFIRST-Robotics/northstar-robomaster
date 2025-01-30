/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_POSITION_INTERFACE_HPP_
#define TURRET_POSITION_INTERFACE_HPP_

#include <stdint.h>

#include "modm/math/geometry/vector3.hpp"

namespace control::turret
{
/**
 * An interface that provides turret world yaw and pitch.
 *
 * All angles computed using a right hand coordinate system. In other words, yaw is a value from
 * 0-M_TWOPI rotated counterclockwise when looking at the turret from above. Pitch is a value from
 * 0-M_TWOPI rotated counterclockwise when looking at the turret from the right side of the turret.
 */
class TurretPositionInterface
{
public:
    /**
     * @return An angle between [0, M_TWOPI]. The heading of the robot is defined by the turret
     * rather than the traditional case where the robot's heading is defined by the chassis. This 
     * is because the chassis doesn't have a gyro, so the turret's heading is used as a proxy.
     */
    virtual inline float getTurretHeading() = 0;


    virtual inline float getChassisHeading() = 0;
    
    /**
     * @return An angle between [0, M_TWOPI] that is the chassis-relative angle of the
     * turret counterclockwise when looking at the turret from the right side.
     */
    virtual inline float getTurretPitch() = 0;

};  // class TurretOrientation

}  // namespace control::turret

#endif  // TURRET_ORIENTATION_INTERFACE_HPP_
