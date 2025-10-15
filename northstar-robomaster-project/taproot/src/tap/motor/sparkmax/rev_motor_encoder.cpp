#include "rev_motor_encoder.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/motor/sparkmax/rev_motor.hpp"

namespace tap
{
namespace motor
{
RevMotorEncoder::RevMotorEncoder(bool isInverted, float gearRatio, uint32_t encoderHomePosition)
    : WrappedEncoder(isInverted, ENC_RESOLUTION, gearRatio, encoderHomePosition),
      shaftRPM(0)
{
    encoderDisconnectTimeout.stop();
}

void RevMotorEncoder::processMessage(const modm::can::Message& message)
{
    encoderDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME);

    uint32_t period = message.getIdentifier() & ~0xF;

    if (period == 2051840)
    {
        std::memcpy(&shaftRPM, message.data, sizeof(float));
    }
    else if (period == 2051880)
    {
        float encoderPos;  // REV sends this in rotations
        std::memcpy(&encoderPos, message.data, sizeof(encoderPos));

        uint16_t encoderActual = static_cast<uint16_t>(std::llround(encoderPos * 42.0f)) % 42 + 1;

        updateEncoderValue(encoderActual);
    }
}

bool RevMotorEncoder::isOnline() const
{
    /*
     * motor online if the disconnect timout has not expired (if it received message but
     * somehow got disconnected) and the timeout hasn't been stopped (initially, the timeout
     * is stopped)
     */
    return !encoderDisconnectTimeout.isExpired() && !encoderDisconnectTimeout.isStopped();
}

float RevMotorEncoder::getVelocity() const
{
    return this->getShaftRPM() * static_cast<float>(M_TWOPI) / 60.f * this->gearRatio;
}

int16_t RevMotorEncoder::getShaftRPM() const { return shaftRPM; }
}  // namespace motor

}  // namespace tap
