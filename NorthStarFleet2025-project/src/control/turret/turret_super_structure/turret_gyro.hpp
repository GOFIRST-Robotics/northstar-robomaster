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

#ifndef TURRET_GYRO_HPP_
#define TURRET_GYRO_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/can/can_rx_listener.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/math/geometry/angle.hpp"

#include "tap/algorithms/wrapped_float.hpp"



namespace src
{
class Drivers;
}

namespace control::turret
{
/**
 * Isaac: constructs a wrapper object for the gyro that currently stores a 
 * little bit of additional information. useful later if I decide I need
 * to add the velocities which I currently am not using but can easily be 
 * added if I want to later.  
*/
class TurretMCBCGryo
{
public:
    TurretMCBCGryo(tap::Drivers* drivers);
    /**
     * @return turret yaw angle in radians, normalized between [-pi, pi]
     */
    float getRoll();
    /**
     * @return An unwrapped (not normalized) turret yaw angle, in rad. This object keeps track of
     * the number of revolutions that the attached turret IMU has taken, and the number of
     * revolutions is reset once the IMU is recalibrated or if the turret IMU comes disconnected.
     */
    float getRollUnwrapped();

    /**
     * @return turret pitch angle in rad, a value normalized between [-pi, pi]
     */
    float getPitch();

    /**
     * @return An unwrapped (not normalized) turret pitch angle, in rad. This object keeps track of
     * the number of revolutions that the attached turret IMU has taken, and the number of
     * revolutions is reset once the IMU is recalibrated or if the turret IMU comes disconnected.
     */
    float getPitchUnwrapped();

    /**
     * @return turret yaw angle in radians, normalized between [-pi, pi]
     */
     float getYaw();

     /**
     * @return turret yaw angle in radians, normalized between [-pi, pi]
     */
     tap::algorithms::WrappedFloat getYawWrapped();

    /**
     * @return An unwrapped (not normalized) turret yaw angle, in rad. This object keeps track of
     * the number of revolutions that the attached turret IMU has taken, and the number of
     * revolutions is reset once the IMU is recalibrated or if the turret IMU comes disconnected.
     */
    float getYawUnwrapped();

private:
    tap::Drivers* drivers;
    float currYaw = 0;
    float prevYaw = 0;
    float currPitch = 0;
    float prevPitch = 0;
    float currRoll = 0;
    float prevRoll = 0;
    int yawRevolutions = 0;
    int pitchRevolutions = 0;
    int rollRevolutions = 0;

    /**
     * Updates the passed in revolutionCounter if a revolution increment or decrement has been
     * detected.
     *
     * A revolution increment is detected if the difference between the new and old angle is < -pi,
     * and a decrement is detected if the difference is > pi. Put simply, if the angle measurement
     * jumped unexpectly, it is assumed that a revolution has ocurred.
     *
     * @param[in] newAngle A new angle measurement, in radians.
     * @param[in] prevAngle The old (previous) angle measurement, in radians.
     * @param[out] revolutionCounter Counter to update, either unchanged, incremented, or
     * decremented based on newAngle and prevAngle's state.
     */
    static inline void updateRevolutionCounter(
        const float newAngle,
        const float prevAngle,
        int& revolutionCounter)
    {
        const float angleDiff = newAngle - prevAngle;
        if (angleDiff < -M_PI)
        {
            revolutionCounter++;
        }
        else if (angleDiff > M_PI)
        {
            revolutionCounter--;
        }
    }
};
}  // namespace aruwsrc::can

#endif  // TURRET_MCB_CAN_COMM_HPP_
