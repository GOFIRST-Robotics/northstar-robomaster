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

void RevMotorEncoder::processMessage(const modm::can::Message& message, APICommand period)
{
    encoderDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME);

    uint32_t receivedArbId = message.getIdentifier();
    uint64_t rawValue = 0;
    std::memcpy(&rawValue, message.data, sizeof(uint64_t));

    if (period == APICommand::Period1)
    {
        shaftRPM = rawValue & 0xFFFFFFFF;
    }
    else if (period == APICommand::Period2)
    {
        uint16_t encoderActual = rawValue & 0xFFFFFFFF;
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
