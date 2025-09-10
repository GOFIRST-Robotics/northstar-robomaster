#ifndef TARGET_HERO

#ifndef FLYWHEEL_SUBSYSTEM_HPP_
#define FLYWHEEL_SUBSYSTEM_HPP_

#include <modm/container/pair.hpp>

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/sparkmax/rev_motor.hpp"

#include "control/flywheel/flywheel_constants.hpp"
#include "modm/math/filter/pid.hpp"

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

    mockable void setDesiredSpin(u_int16_t spin);

    mockable float getDesiredSpin() const { return desiredSpin; }

    mockable void setDesiredLaunchSpeed(float speed);

    mockable float getDesiredLaunchSpeedLeft() const { return desiredLaunchSpeedLeft; }
    mockable float getDesiredLaunchSpeedRight() const { return desiredLaunchSpeedRight; }
    mockable float getDesiredLaunchSpeedUp() const { return desiredLaunchSpeedUp; }

    mockable float getDesiredFlywheelSpeedLeft() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft);
    }
    mockable float getDesiredFlywheelSpeedRight() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedRight);
    }
    mockable float getDesiredFlywheelSpeedUp() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedUp);
    }

    float getCurrentLeftFlywheelMotorRPM() const { return leftWheel.getVelocity(); }

    float getCurrentRightFlywheelMotorRPM() const { return rightWheel.getVelocity(); }

    float getCurrentUpFlywheelMotorRPM() const { return upWheel.getVelocity(); }

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        leftWheel.setControlValue(0);  // TODO CHANGE
        rightWheel.setControlValue(0);
        upWheel.setControlValue(0);
    }

    const char *getName() const override { return "Flywheels"; }

protected:
    static constexpr float MAX_DESIRED_LAUNCH_SPEED = 8000;  // TODO

    tap::Drivers *drivers;

private:
    modm::Pid<float> velocityPidLeftWheel;
    modm::Pid<float> velocityPidRightWheel;
    modm::Pid<float> velocityPidUpWheel;

    float desiredLaunchSpeedLeft;
    float desiredLaunchSpeedRight;
    float desiredLaunchSpeedUp;

    Spin desiredSpin = SPIN_100;
    u_int16_t desiredSpinValue = 100;  // percent of spin

    uint32_t prevTime = 0;

    tap::algorithms::Ramp desiredRpmRampLeft;
    tap::algorithms::Ramp desiredRpmRampRight;
    tap::algorithms::Ramp desiredRpmRampUp;

    tap::motor::RevMotor leftWheel;
    tap::motor::RevMotor rightWheel;
    tap::motor::RevMotor upWheel;

    float launchSpeedToFlywheelRpm(float launchSpeed) const;

    std::array<std::array<modm::Pair<float, float>, 4>, SPIN_COUNT> spinToRPMMap;
    std::array<std::array<modm::Pair<float, float>, 4>, SPIN_COUNT> feedforwardmap;

    modm::Pid<float> topFlyWheelPid;
    modm::Pid<float> bottomLeftFlyWheelPid;
    modm::Pid<float> bottomRightFlyWheelPid;

    float previousTopSetPoint = 0.0f;
    float previousLeftSetPoint = 0.0f;
    float previousRightSetPoint = 0.0f;

    float previousLaunchSpeedLeft = 0.0f;
    float previousLaunchSpeedRight = 0.0f;
    float previousLaunchSpeedUp = 0.0f;
};

}  // namespace src::control::flywheel

#endif

#endif