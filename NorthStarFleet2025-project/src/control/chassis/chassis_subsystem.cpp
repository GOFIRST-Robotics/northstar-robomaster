#include "chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "drivers.hpp"

#define THING

#include <cmath>

using tap::algorithms::limitVal;

namespace control::chassis
{
// STEP 1 (Tank Drive): create constructor
    ChassisSubsystem::ChassisSubsystem(tap::Drivers& drivers, const ChassisConfig& config) :
    Subsystem(&drivers), 
    desiredOutput{},
    pidControllers{},
    motors{
        Motor(&drivers, config.leftFrontId, config.canBus, false, "LF"),
        Motor(&drivers, config.leftBackId, config.canBus, false, "LB"),
        Motor(&drivers, config.rightFrontId, config.canBus, true, "RF"),
        Motor(&drivers, config.rightBackId, config.canBus, true, "RB"),
    }
    {
        for (auto &controller : pidControllers) {
            controller.setParameter(config.wheelVelocityPidConfig);
        }
    }
// STEP 2 (Tank Drive): initialize function
    void ChassisSubsystem::initialize() {
        for (auto &i : motors) {
            i.initialize();
        }
    }
// STEP 3 (Tank Drive): setVelocityTankDrive function
    void ChassisSubsystem::setVelocityTankDrive(float forward, float sideways, float rotational, float robotHeading) {
        float distToCenter;
        float LFSpeed;
        float LBSpeed;
        float RFSpeed;
        float RBSpeed;
        #ifdef THING //TODO Make not THING
        distToCenter = 10.0f; // In inches atm
        float forwardAdjusted = forward * cos(robotHeading);
        float sidewaysAdjusted = sideways * sin(robotHeading);
        LFSpeed = mpsToRpm(forwardAdjusted - sidewaysAdjusted - (2 * distToCenter * rotational * M_PI / 180));
        LBSpeed = mpsToRpm(forwardAdjusted + sidewaysAdjusted - (2 * distToCenter * rotational * M_PI / 180));
        RFSpeed = mpsToRpm(forwardAdjusted + sidewaysAdjusted + (2 * distToCenter * rotational * M_PI / 180));
        RBSpeed = mpsToRpm(forwardAdjusted - sidewaysAdjusted + (2 * distToCenter * rotational * M_PI / 180));
        #else
        distToCenter = 10.0f;
        LFSpeed = mpsToRpm(forward * cos(robotHeading) + sideways * sin(robotHeading) + modm::toRadian(rotational) * distToCenter);
        RFSpeed = mpsToRpm(forward * cos(robotHeading + M_PI_2) + sideways * sin(robotHeading + M_PI_2) + modm::toRadian(rotational) * distToCenter);
        RBSpeed = mpsToRpm(forward * cos(robotHeading + M_PI) + sideways * sin(robotHeading + M_PI) + modm::toRadian(rotational) * distToCenter);
        LBSpeed = mpsToRpm(forward * cos(robotHeading + 3 * M_PI / 2) + sideways * sin(robotHeading + 3 * M_PI / 2) + modm::toRadian(rotational) * distToCenter);
        #endif
        desiredOutput[static_cast<int>(MotorId::LF)] = limitVal<float>(LFSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
        desiredOutput[static_cast<int>(MotorId::LB)] = limitVal<float>(LBSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
        desiredOutput[static_cast<int>(MotorId::RF)] = limitVal<float>(RFSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
        desiredOutput[static_cast<int>(MotorId::RB)] = limitVal<float>(RBSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    }
    void ChassisSubsystem::refresh() {
        auto runPid = [](Pid &pid, Motor &motor, float desiredOutput) {
            pid.update(desiredOutput - motor.getShaftRPM());
            motor.setDesiredOutput(pid.getValue());
        };

        for (size_t ii = 0; ii < motors.size(); ii++)
        {
            runPid(pidControllers[ii], motors[ii], desiredOutput[ii]);
        }
    }
}  // namespace control::chassis
