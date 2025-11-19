#ifndef FLYWHEEL_INTERFACE_HPP_
#define FLYWHEEL_INTERFACE_HPP_

namespace src::control::flywheel
{
class FlywheelInterface
{
public:
    FlywheelInterface() = default;

    ~FlywheelInterface() = default;

    virtual void setDesiredLaunchSpeed(float speed) = 0;

    virtual float getDesiredLaunchSpeed() const = 0;

    virtual float getDesiredFlywheelSpeed() const = 0;

    virtual float getCurrentFlywheelAverageMotorRPM() const = 0;

protected:
    static constexpr float MAX_DESIRED_LAUNCH_SPEED_MPS = 30.0f;

private:
    virtual float launchSpeedToFlywheelRpm(float launchSpeed) const = 0;
};

}  // namespace src::control::flywheel

#endif  // FLYWHEEL_INTERFACE_HPP_