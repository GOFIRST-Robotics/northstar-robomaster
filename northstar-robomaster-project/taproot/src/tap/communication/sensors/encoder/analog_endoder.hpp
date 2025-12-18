#ifndef PWM_ENCODER_HPP_
#define PWM_ENCODER_HPP_

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/communication/gpio/analog.hpp"

#include "wrapped_encoder.hpp"

namespace tap::encoder
{
class AnalogEncoder : public WrappedEncoder
{
public:
    static constexpr float ENC_RESOLUTION = 4096.0f;

    AnalogEncoder(
        tap::gpio::Analog& analog,
        tap::gpio::Analog::Pin pin,
        bool isInverted,
        float gearRatio = 1,
        uint32_t encoderHomePosition = 0);

    void initialize() override;

    uint16_t AnalogEncoder::getRawReading();

    bool isOnline() const override;

private:
    tap::gpio::Analog& analog;
    tap::gpio::Analog::Pin pin;
};

}  // namespace tap::encoder

#endif  // TAPROOT_ENCODER_INTERFACE_HPP_
