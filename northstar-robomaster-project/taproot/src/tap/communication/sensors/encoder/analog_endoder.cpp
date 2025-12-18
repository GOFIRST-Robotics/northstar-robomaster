#include "analog_endoder.hpp"

namespace tap::encoder
{
AnalogEncoder::AnalogEncoder(
    tap::gpio::Analog &analog,
    tap::gpio::Analog::Pin pin,
    bool isInverted,
    float gearRatio = 1,
    uint32_t encoderHomePosition = 0)
    : WrappedEncoder(isInverted, ENC_RESOLUTION, gearRatio, encoderHomePosition),
      analog(analog),
      pin(pin)
{
}

void AnalogEncoder::initialize() { analog.init(); };

uint16_t AnalogEncoder::getRawReading()
{
    // 1. Read voltage in millivolts (0mV to ~3300mV)
    uint16_t voltageMv = analog.read(pin);

    // 2. Constraint: Standard logic voltage is 3.3V (3300mV).
    // If the sensor outputs exactly 3.3V, the ADC might read 3305mV due to noise.
    // We clamp it to 3300 to prevent math errors.
    if (voltageMv > 3300)
    {
        voltageMv = 3300;
    }

    // 3. Convert mV to Encoder Ticks
    // Math: (Voltage / MaxVoltage) * MaxResolution
    float fraction = static_cast<float>(voltageMv) / 3300.0f;

    // Return 12-bit integer (0 to 4096)
    updateEncoderValue(static_cast<uint16_t>(fraction * ENC_RESOLUTION));
}

bool AnalogEncoder::isOnline() const { return analog.read(pin) > 10; };

};  // namespace tap::encoder
