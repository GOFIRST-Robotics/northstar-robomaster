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

#include "yaw_turret_motor.hpp"
#include "tap/algorithms/wrapped_float.hpp"


using namespace tap::motor;
using namespace tap::algorithms;


float calibrationOffsetDebug;
float chassisFrameUnwrappedMeasurementDebug;
float smallGearChassisFrameUnwrappedMeasurementDebug;
namespace control::turret
{

    YawTurretMotor::YawTurretMotor(tap::motor::MotorInterface *motor, const TurretMotorConfig &motorConfig, TurretMCBCGryo* turretGyro)
        : TurretMotor(motor,  motorConfig), 
        m_turretGyro(turretGyro) 
    {

    }


    void YawTurretMotor::initialize()
    {
        calibrationOffset = M_PI_2;
        // motor->resetEncoderValue();
    }

    //fromt the perspective you are controlling the driven gear rather than the driving gear
    float YawTurretMotor::getChassisFrameUnwrappedMeasuredAngle() const {
        smallGearChassisFrameUnwrappedMeasurementDebug = chassisFrameUnwrappedMeasurement;
        chassisFrameUnwrappedMeasurementDebug = chassisFrameUnwrappedMeasurement * 3/4 + calibrationOffset;
       return chassisFrameUnwrappedMeasurement * 3/4 + calibrationOffset;
    }

    float YawTurretMotor::getChassisDriveOffset() const {
         return WrappedFloat(getChassisFrameUnwrappedMeasuredAngle(), 0, M_TWOPI).getWrappedValue();
    }

    float m_turretGyroDebug;
    float YawTurretMotor::getHeadingOfTurret() const {
        m_turretGyro->getYaw();
        m_turretGyroDebug = m_turretGyro->getYawUnwrapped();
        return m_turretGyro->getYawUnwrapped();
    }
    

}  
