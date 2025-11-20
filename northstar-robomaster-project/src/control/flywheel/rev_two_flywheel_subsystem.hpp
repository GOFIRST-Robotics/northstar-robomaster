#ifndef REV_TWO_FLYWHEEL_SUBSYSTEM_HPP_
#define REV_TWO_FLYWHEEL_SUBSYSTEM_HPP_

#include <modm/container/pair.hpp>

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/sparkmax/rev_motor.hpp"

#include "control/flywheel/flywheel_constants.hpp"
#include "modm/math/filter/pid.hpp"

#include "two_flywheel_subsystem.hpp"

namespace src::control::flywheel
{
class RevTwoFlywheelSubsystem : public TwoFlywheelSubsystem
{
public:
    RevTwoFlywheelSubsystem(
        tap::Drivers *drivers,
        tap::motor::REVMotorId leftMotorId,
        tap::motor::REVMotorId rightMotorId,
        tap::can::CanBus canBus);

    void initialize() override;

    void setDesiredLaunchSpeed(float speed) override;

    float getDesiredLaunchSpeedLeft() const { return desiredLaunchSpeedLeft; }
    float getDesiredLaunchSpeedRight() const { return desiredLaunchSpeedRight; }

    float getDesiredLaunchSpeed() const override
    {
        return (desiredLaunchSpeedLeft + desiredLaunchSpeedRight) / 2.0f;
    }

    float getDesiredFlywheelSpeedLeft() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft);
    }
    float getDesiredFlywheelSpeedRight() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedRight);
    }

    float getDesiredFlywheelSpeed() const override
    {
        return (getDesiredFlywheelSpeedLeft() + getDesiredFlywheelSpeedRight()) / 2.0f;
    }

    float getCurrentLeftFlywheelMotorRPM() const
    {
        return leftWheel.getEncoder()->getVelocity() * 60 / M_TWOPI;
    }

    float getCurrentRightFlywheelMotorRPM() const
    {
        return rightWheel.getEncoder()->getVelocity() * 60 / M_TWOPI;
    }

    float getCurrentFlywheelAverageMotorRPM() const override
    {
        return (getCurrentLeftFlywheelMotorRPM() + getCurrentRightFlywheelMotorRPM()) / 2.0f;
    }

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        leftWheel.setControlValue(0);
        rightWheel.setControlValue(0);
    }

    const char *getName() const override { return "Flywheels"; }

private:
    tap::motor::RevMotor::PIDConfig pidConfig;

    float desiredLaunchSpeedLeft;
    float desiredLaunchSpeedRight;

    uint32_t prevTime = 0;

    tap::algorithms::Ramp desiredRpmRampLeft;
    tap::algorithms::Ramp desiredRpmRampRight;

    tap::motor::RevMotor leftWheel;
    tap::motor::RevMotor rightWheel;

    float launchSpeedToFlywheelRpm(float launchSpeed) const override;
};

}  // namespace src::control::flywheel

#endif  // REV_TWO_FLYWHEEL_SUBSYSTEM_HPP_