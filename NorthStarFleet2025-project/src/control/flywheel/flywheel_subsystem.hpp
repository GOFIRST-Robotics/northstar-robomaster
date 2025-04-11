#ifndef FLYWHEEL_SUBSYSTEM_HPP_
#define FLYWHEEL_SUBSYSTEM_HPP_

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/rev_motor.hpp"

#include "modm/math/filter/pid.hpp"

namespace src
{
class Drivers;
}

namespace src::control::flywheel
{
class FlywheelSubsystem : public tap::control::Subsystem
{
public:
    FlywheelSubsystem(
        tap::Drivers *drivers,
        tap::motor::REVMotorId leftMotorId,
        tap::motor::REVMotorId rightMotorId,
        tap::motor::REVMotorId upMotorId,
        tap::can::CanBus canBus);

    void initialize() override;

    mockable void setDesiredLaunchSpeed(float speed);

    mockable float getDesiredLaunchSpeed() const { return desiredLaunchSpeed; }

    float getCurrentFlyWheelMotorRPM(tap::motor::RevMotor motor) const;

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        leftWheel.setTargetVoltage(0);  // TODO CHANGE
        rightWheel.setTargetVoltage(0);
        upWheel.setTargetVoltage(0);
    }

    const char *getName() const override { return "Flywheels"; }

protected:
    static constexpr float MAX_DESIRED_LAUNCH_SPEED = 5000;  // TODO

    tap::Drivers *drivers;

private:
    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    modm::Pid<float> velocityPidUpWheel;

    float desiredLaunchSpeed;

    uint32_t prevTime = 0;

    tap::algorithms::Ramp desiredRpmRamp;

    tap::motor::RevMotor leftWheel;
    tap::motor::RevMotor rightWheel;
    tap::motor::RevMotor upWheel;

    float launchSpeedToFlywheelRpm(float launchSpeed) const;
};

}  // namespace src::control::flywheel

#endif
