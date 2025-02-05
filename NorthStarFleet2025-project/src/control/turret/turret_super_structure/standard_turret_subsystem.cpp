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

#include "standard_turret_subsystem.hpp"
#include "tap/algorithms/wrapped_float.hpp"

using namespace tap::algorithms;
namespace control::turret

{

float StandardTurretSubsystem::getTurretHeading() { return getTurretGyro()->getYaw(); }

float StandardTurretSubsystem::getTurretPitch() { return getTurretGyro()->getPitch(); }
// float StandardTurretSubsystem::getTurretPitch() { return 0.05; }

float StandardTurretSubsystem::getChassisHeading() {
    WrappedFloat turretHeading = WrappedFloat(getTurretHeading(), 0, M_TWOPI);
    WrappedFloat yawMotorWrapped = yawMotor.getChassisFrameMeasuredAngle();
    return turretHeading.minDifference(yawMotorWrapped);
}

}  // namespace aruwsrc::control::turret
