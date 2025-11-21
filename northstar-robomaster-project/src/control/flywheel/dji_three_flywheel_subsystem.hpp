#ifndef DJI_THREE_FLYWHEEL_SUBSYSTEM_HPP
#define DJI_THREE_FLYWHEEL_SUBSYSTEM_HPP

#include <modm/container/pair.hpp>

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#include "control/flywheel/flywheel_constants.hpp"
#include "modm/math/filter/pid.hpp"

#include "three_flywheel_subsystem.hpp"

namespace src::control::flywheel
{
class DJIThreeFlywheelSubsystem : public ThreeFlywheelSubsystem
{
public:
    DJIThreeFlywheelSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorId leftMotorId,
        tap::motor::MotorId rightMotorId,
        tap::motor::MotorId downMotorId,
        tap::can::CanBus canBus);

    void initialize() override;

    void setDesiredSpin(u_int16_t spin) override;

    float getDesiredSpin() const override { return desiredSpin; }

    void setDesiredLaunchSpeed(float speed) override;

    void setDesiredFlywheelSpeed(float rpm) override;

    float getDesiredLaunchSpeedLeft() const { return desiredLaunchSpeedLeft; }
    float getDesiredLaunchSpeedRight() const { return desiredLaunchSpeedRight; }
    float getDesiredLaunchSpeedDown() const { return desiredLaunchSpeedDown; }

    float getDesiredLaunchSpeed() const override
    {
        return (desiredLaunchSpeedLeft + desiredLaunchSpeedRight + desiredLaunchSpeedDown) / 3.0f;
    }

    float getDesiredFlywheelSpeedLeft() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft);
    }
    float getDesiredFlywheelSpeedRight() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedRight);
    }
    float getDesiredFlywheelSpeedDown() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedDown);
    }

    float getDesiredFlywheelSpeed() const override
    {
        return (getDesiredFlywheelSpeedLeft() + getDesiredFlywheelSpeedRight() +
                getDesiredFlywheelSpeedDown()) /
               3.0f;
    }

    float getCurrentLeftFlywheelMotorRPM() const { return getWheelRPM(&leftWheel); }

    float getCurrentRightFlywheelMotorRPM() const { return getWheelRPM(&rightWheel); }

    float getCurrentDownFlywheelMotorRPM() const { return getWheelRPM(&downWheel); }

    float getCurrentFlywheelAverageMotorRPM() const override
    {
        return (getCurrentLeftFlywheelMotorRPM() + getCurrentRightFlywheelMotorRPM() +
                getCurrentDownFlywheelMotorRPM()) /
               3.0f;
    }

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        leftWheel.setDesiredOutput(0);  // TODO CHANGE
        rightWheel.setDesiredOutput(0);
        downWheel.setDesiredOutput(0);
    }

    const char *getName() const override { return "Flywheels"; }

private:
    modm::Pid<float> velocityPidLeftWheel;
    modm::Pid<float> velocityPidRightWheel;
    modm::Pid<float> velocityPidDownWheel;

    float desiredLaunchSpeedLeft;
    float desiredLaunchSpeedRight;
    float desiredLaunchSpeedDown;

    uint32_t prevTime = 0;

    tap::algorithms::Ramp desiredRpmRampLeft;
    tap::algorithms::Ramp desiredRpmRampRight;
    tap::algorithms::Ramp desiredRpmRampDown;

    tap::motor::DjiMotor leftWheel;
    tap::motor::DjiMotor rightWheel;
    tap::motor::DjiMotor downWheel;

    float launchSpeedToFlywheelRpm(float launchSpeed) const override;

    float getWheelRPM(const tap::motor::DjiMotor *motor) const
    {
        return motor->getEncoder()->getVelocity() * 60.0f / M_TWOPI;
    }
};

}  // namespace src::control::flywheel

#endif  // HERO_FLYWHEEL_SUBSYSTEM
