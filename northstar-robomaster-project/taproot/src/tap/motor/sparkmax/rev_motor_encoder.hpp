#ifndef REV_MOTOR_ENCODER_HPP_
#define REV_MOTOR_ENCODER_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/communication/sensors/encoder/wrapped_encoder.hpp"
#include "tap/util_macros.hpp"

#include "modm/architecture/interface/can_message.hpp"
#include "modm/math/geometry/angle.hpp"

namespace tap::motor
{
enum class APICommand : uint16_t;
/**
 * A class designed to interface with the encoder for REV brand motors and motor controllers
 * over CAN.
 * @note: the default positive rotation direction (i.e.: when `this->isMotorInverted()
 *      == false`) is counter clockwise when looking at the shaft from the side opposite
 *      the motor.
 */
class RevMotorEncoder : public tap::encoder::WrappedEncoder
{
public:
    static constexpr uint16_t ENC_RESOLUTION = 42;

    /**
     * @param isInverted if `false` the positive rotation direction of the shaft is
     *      counter-clockwise when looking at the shaft from.
     *      If `true` then the positive rotation direction will be clockwise.
     * @param encoderResolution the number of encoder ticks before the value wraps.
     * @param gearRatio the ratio of input revolutions to output revolutions of this encoder.
     * @param encoderHomePosition the zero position for the encoder in encoder ticks.
     */
    RevMotorEncoder(bool isInverted, float gearRatio = 1, uint32_t encoderHomePosition = 0);

    void initialize() override{};

    bool isOnline() const override;

    float getVelocity() const override;

    /**
     * The current RPM reported by the motor controller.
     */
    mockable int16_t getShaftRPM() const;

    DISALLOW_COPY_AND_ASSIGN(RevMotorEncoder)

    /**
     * Overrides virtual method in the can class, called every time a message with the
     * CAN message id this class is attached to is received by the can receive handler.
     * Parses the data in the message and updates this class's fields accordingly.
     *
     * @param[in] message the message to be processed.
     * @param[in] period the period for velo or position.
     */
    mockable void processMessage(const modm::can::Message& message, APICommand period);

private:
    // wait time before the motor is considered disconnected, in milliseconds
    static const uint32_t MOTOR_DISCONNECT_TIME = 100;

    tap::arch::MilliTimeout encoderDisconnectTimeout;

    int16_t shaftRPM;
};

}  // namespace tap::motor

#endif  // REV_MOTOR_ENCODER_HPP_
