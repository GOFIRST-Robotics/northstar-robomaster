#ifndef HOPPER_SUBSYSTEM_HPP_
#define HOPPER_SUBSYSTEM_HPP_

#include "tap/communication/gpio/pwm.hpp"
#include "tap/control/subsystem.hpp"


namespace src::control::hopper
{
class HopperSubsystem : public tap::control::Subsystem
{
public:
    HopperSubsystem(tap::Drivers* drivers, float minPwm, float maxPwm, tap::gpio::Pwm::Pin pwmPin);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override {}

    const char* getName() const override { return "hopper subsystem"; }

    void setPWM(float dutyCycle);

    void open();

    void close();

private:
    uint32_t prevTime = 0;

    tap::gpio::Pwm::Pin pwmPin;

    float minPwm;

    float maxPwm;

    float pwm;

    float velocitySetpoint = 0;

    bool isOpen = false;
};

}  // namespace src::control::hopper

#endif  // HERO_AGITATOR_SUBSYSTEM_HPP_