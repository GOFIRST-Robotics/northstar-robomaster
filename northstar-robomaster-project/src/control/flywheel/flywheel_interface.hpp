#ifndef FLYWHEEL_INTERFACE_HPP_
#define FLYWHEEL_INTERFACE_HPP_

namespace src::control::flywheel
{
class FlywheelInterface
{
public:
    FlywheelInterface() = default;

    ~FlywheelInterface() = default;

    // MPS
    virtual void setDesiredLaunchSpeed(float speed) = 0;
    // MPS
    virtual float getDesiredLaunchSpeed() const = 0;
    // RPM
    virtual void setDesiredFlywheelSpeed(float rpm) = 0;
    // RPM
    virtual float getDesiredFlywheelSpeed() const = 0;
    // RPM
    virtual float getCurrentFlywheelAverageMotorRPM() const = 0;

protected:
    static constexpr float MAX_DESIRED_LAUNCH_SPEED_MPS = 30.0f;

private:
    virtual float launchSpeedToFlywheelRpm(float launchSpeed) const = 0;
};

}  // namespace src::control::flywheel

#endif  // FLYWHEEL_INTERFACE_HPP_