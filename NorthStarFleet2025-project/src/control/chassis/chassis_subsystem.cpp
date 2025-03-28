#include "chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "tap/drivers.hpp"

//#define MECANUM

#include <cmath>

using tap::algorithms::limitVal;

namespace control::chassis
{
// STEP 1 (Tank Drive): create constructor

    float stupidHead;

    ChassisSubsystem::ChassisSubsystem(tap::Drivers* drivers, const ChassisConfig& config, src::can::TurretMCBCanComm* turretMcbCanComm) :
    Subsystem(drivers), 
    desiredOutput{},
    pidControllers{},
    motors{
        Motor(drivers, config.leftFrontId, config.canBus, false, "LF"),
        Motor(drivers, config.leftBackId, config.canBus, false, "LB"),
        Motor(drivers, config.rightFrontId, config.canBus, false, "RF"),
        Motor(drivers, config.rightBackId, config.canBus, false, "RB"),
    },
    rateLimiters{
        control::chassis::algorithms::SlewRateLimiter(1000, 10),
        control::chassis::algorithms::SlewRateLimiter(1000, 10),
        control::chassis::algorithms::SlewRateLimiter(1000, 10),
        control::chassis::algorithms::SlewRateLimiter(1000, 10),
    },
    turretMcbCanComm(turretMcbCanComm)
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
    float debugLB;
    float debugdesiredOutput;
    float debugpid;
// STEP 3 (Tank Drive): setVelocityTankDrive function
    void ChassisSubsystem::setVelocityDrive(float forward, float sideways, float rotational, float turretRot = 0.0f) {
        float distToCenter;
        forward = 1.0;
        
        // drivers->bmi088.read();
        #ifdef FIELD
        float robotHeading = modm::toRadian(turretMcbCanComm->getYaw());
        // robotHeading = fmod(robotHeading, 2 * M_PI);
        #else
        float robotHeading = -(turretRot); // Signs subject to change, just want the difference
        robotHeading = fmod(robotHeading, 2 * M_PI);
        #endif
        // For robotCentric only just + M_PI_4
        #ifdef MECANUM
        //Mecanum
        distToCenter = 10.0f; // In inches atm
        float forwardAdjusted = forward * cos(robotHeading);
        float sidewaysAdjusted = sideways * sin(robotHeading);
        LFSpeed = mpsToRpm(forwardAdjusted - sidewaysAdjusted - (2 * distToCenter * rotational * M_PI / 180));
        LBSpeed = mpsToRpm(forwardAdjusted + sidewaysAdjusted - (2 * distToCenter * rotational * M_PI / 180));
        RFSpeed = mpsToRpm(forwardAdjusted + sidewaysAdjusted + (2 * distToCenter * rotational * M_PI / 180));
        RBSpeed = mpsToRpm(forwardAdjusted - sidewaysAdjusted + (2 * distToCenter * rotational * M_PI / 180));
        #else
        //Omni
        distToCenter = 30.48f;
        turretRot=-turretMcbCanComm->getYaw()+modm::toRadian(drivers->bmi088.getYaw());
        double cos_theta = cos(turretRot);
        double sin_theta = sin(turretRot);
        double vx_local = forward * cos_theta + sideways * sin_theta;
        double vy_local = -forward * sin_theta + sideways * cos_theta;
        double sqrt2 = sqrt(2.0);
        rotational=modm::toRadian(rotational);
        float LFSpeed = mpsToRpm((vx_local - vy_local) / sqrt2 + rotational * distToCenter * sqrt2);  // Front-left wheel
        float RFSpeed = mpsToRpm((-vx_local - vy_local) / sqrt2 + rotational * distToCenter * sqrt2); // Front-right wheel
        float RBSpeed = mpsToRpm((-vx_local + vy_local) / sqrt2 + rotational * distToCenter * sqrt2); // Rear-right wheel
        float LBSpeed = mpsToRpm((vx_local + vy_local) / sqrt2 + rotational * distToCenter * sqrt2);  // Rear-left wheel
        #endif
        int LF = static_cast<int>(MotorId::LF);
        int LB = static_cast<int>(MotorId::LB);
        int RF = static_cast<int>(MotorId::RF);
        int RB = static_cast<int>(MotorId::RB);
        desiredOutput[LF] = limitVal<float>(LFSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
        desiredOutput[LB] = limitVal<float>(LBSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
        desiredOutput[RF] = limitVal<float>(RFSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
        desiredOutput[RB] = limitVal<float>(RBSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
        debugLB = desiredOutput[LB];
    }

    void ChassisSubsystem::refresh() {
        auto runPid = [](Pid &pid, Motor &motor, float desiredOutput) {
            debugdesiredOutput = desiredOutput;
            pid.update(desiredOutput - motor.getShaftRPM());
            debugpid = pid.getValue();
            motor.setDesiredOutput(pid.getValue());
        };

        for (size_t ii = 0; ii < motors.size(); ii++)
        {
            runPid(pidControllers[ii], motors[ii], desiredOutput[ii]);
        }
    }
}  // namespace control::chassis
