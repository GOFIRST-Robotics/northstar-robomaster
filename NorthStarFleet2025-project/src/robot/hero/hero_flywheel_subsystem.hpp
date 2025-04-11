#ifndef HERO_FLYWHEEL_SUBSYSTEM
#define HERO_FLYWHEEL_SUBSYSTEM

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#include "modm/math/filter/pid.hpp"

namespace src::control::flywheel
{
class HeroFlywheelSubsystem : public tap::control::Subsystem
{
public:
    HeroFlywheelSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorId leftMotorId,
        tap::motor::MotorId rightMotorId,
        tap::motor::MotorId upMotorId,
        tap::can::CanBus canBus);

    void initialize() override;

    mockable void setDesiredLaunchSpeed(float speed);

    mockable float getDesiredLaunchSpeed() const { return desiredLaunchSpeed; }

    float getCurrentFlyWheelMotorRPM(tap::motor::DjiMotor motor) const;

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        leftWheel.setDesiredOutput(0);  // TODO CHANGE
        rightWheel.setDesiredOutput(0);
        upWheel.setDesiredOutput(0);
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

    tap::motor::DjiMotor leftWheel;
    tap::motor::DjiMotor rightWheel;
    tap::motor::DjiMotor upWheel;

    float launchSpeedToFlywheelRpm(float launchSpeed) const;
};

}  // namespace src::control::flywheel

#endif