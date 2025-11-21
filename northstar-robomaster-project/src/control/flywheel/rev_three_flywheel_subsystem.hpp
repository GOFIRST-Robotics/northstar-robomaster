#ifndef REV_THREE_FLYWHEEL_SUBSYSTEM_HPP_
#define REV_THREE_FLYWHEEL_SUBSYSTEM_HPP_

#include <modm/container/pair.hpp>

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/sparkmax/rev_motor.hpp"

#include "control/flywheel/flywheel_constants.hpp"
#include "modm/math/filter/pid.hpp"

#include "three_flywheel_subsystem.hpp"

namespace src::control::flywheel
{
class RevThreeFlywheelSubsystem : public ThreeFlywheelSubsystem
{
public:
    RevThreeFlywheelSubsystem(
        tap::Drivers *drivers,
        tap::motor::REVMotorId leftMotorId,
        tap::motor::REVMotorId rightMotorId,
        tap::motor::REVMotorId upMotorId,
        tap::can::CanBus canBus);

    void initialize() override;

    void setDesiredSpin(u_int16_t spin) override;

    float getDesiredSpin() const override { return desiredSpin; }

    void setDesiredLaunchSpeed(float speed) override;

    void setDesiredFlywheelSpeed(float rpm) override;

    float getDesiredLaunchSpeedLeft() const { return desiredLaunchSpeedLeft; }
    float getDesiredLaunchSpeedRight() const { return desiredLaunchSpeedRight; }
    float getDesiredLaunchSpeedUp() const { return desiredLaunchSpeedUp; }

    float getDesiredLaunchSpeed() const override
    {
        return (desiredLaunchSpeedLeft + desiredLaunchSpeedRight + desiredLaunchSpeedUp) / 3.0f;
    }

    float getDesiredFlywheelSpeedLeft() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft);
    }
    float getDesiredFlywheelSpeedRight() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedRight);
    }
    float getDesiredFlywheelSpeedUp() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedUp);
    }

    float getDesiredFlywheelSpeed() const override
    {
        return (getDesiredFlywheelSpeedLeft() + getDesiredFlywheelSpeedRight() +
                getDesiredFlywheelSpeedUp()) /
               3.0f;
    }

    float getCurrentLeftFlywheelMotorRPM() const
    {
        return leftWheel.getEncoder()->getVelocity() * 60 / M_TWOPI;
    }

    float getCurrentRightFlywheelMotorRPM() const
    {
        return rightWheel.getEncoder()->getVelocity() * 60 / M_TWOPI;
    }

    float getCurrentUpFlywheelMotorRPM() const
    {
        return upWheel.getEncoder()->getVelocity() * 60 / M_TWOPI;
    }

    float getCurrentFlywheelAverageMotorRPM() const override
    {
        return (getCurrentLeftFlywheelMotorRPM() + getCurrentRightFlywheelMotorRPM() +
                getCurrentUpFlywheelMotorRPM()) /
               3.0f;
    }

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        leftWheel.setControlValue(0);
        rightWheel.setControlValue(0);
        upWheel.setControlValue(0);
    }

    const char *getName() const override { return "Flywheels"; }

private:
    tap::motor::RevMotor::PIDConfig pidConfig;

    float desiredLaunchSpeedLeft;
    float desiredLaunchSpeedRight;
    float desiredLaunchSpeedUp;

    uint32_t prevTime = 0;

    tap::algorithms::Ramp desiredRpmRampLeft;
    tap::algorithms::Ramp desiredRpmRampRight;
    tap::algorithms::Ramp desiredRpmRampUp;

    tap::motor::RevMotor leftWheel;
    tap::motor::RevMotor rightWheel;
    tap::motor::RevMotor upWheel;

    float launchSpeedToFlywheelRpm(float launchSpeed) const override;
};  // namespace src::control::flywheel

}  // namespace src::control::flywheel

#endif