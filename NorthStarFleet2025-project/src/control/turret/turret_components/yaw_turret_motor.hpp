/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef YAW_TURRET_MOTOR_HPP_
#define YAW_TURRET_MOTOR_HPP_

#include "control/turret/turret_components/turret_motor.hpp"
#include "control/turret/turret_super_structure/turret_gyro.hpp"

namespace control::turret
{
/**
 * Logic encapsulating the control of a single axis of a turret gimbal motor. Contains logic for
 * storing chassis relative position measurements and setpoints and logic for limiting the angle
 * setpoint.
 *
 * Currently, there are GM6020-specific motor parameters in this object such that it is expected
 * that the gimbal motor used is a 6020, but in general with some taproot-side MRs, this class can
 * be generalized to work with any motor interface.
 */
class YawTurretMotor : public TurretMotor
{
public:
    YawTurretMotor(tap::motor::MotorInterface *motor, const TurretMotorConfig &motorConfig, TurretMCBCGryo* turretGyro);

    void initialize() override;

    float getChassisFrameUnwrappedMeasuredAngle() const override;

    float getChassisDriveOffset() const;

    float getHeadingOfTurret() const;

    float calibrationOffset;

private:
    TurretMCBCGryo* m_turretGyro;
};

}  // namespace control::turret

#endif  // TURRET_MOTOR_HPP_
