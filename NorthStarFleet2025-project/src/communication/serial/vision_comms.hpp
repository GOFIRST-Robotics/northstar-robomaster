#ifndef VISION_COMMS_HPP
#define VISION_COMMS_HPP

#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/drivers.hpp"

#include "control/turret/constants/turret_constants.hpp"

namespace src::serial
{
class VisionComms : public tap::communication::serial::DJISerial
{
public:
    static constexpr size_t VISION_COMMS_BAUD_RATE = 115200;

    static constexpr tap::communication::serial::Uart::UartPort VISION_COMMS_TX_UART_PORT =
        tap::communication::serial::Uart::UartPort::Uart1;
    static constexpr tap::communication::serial::Uart::UartPort VISION_COMMS_RX_UART_PORT =
        tap::communication::serial::Uart::UartPort::Uart1;

    struct TurretAimData
    {
        float yaw;
        float pitch;
        float distance;
        tap::communication::serial::RefSerialData::RobotId robotId;
        float maxErrorYaw;
        float maxErrorPitch;
    };

    struct PlateDims
    {
        float width;
        float height;
    };

    std::unordered_map<int, PlateDims> plateLookup{
        {1, {200.0f, 100.0f}},  // hero plate dimentions in mm
        {7, {100.0f, 100.0f}},  // sentry plate dimentions in mm
        {3, {100.0f, 100.0f}},  // don't know id 3 is correct
    };

    VisionComms(tap::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(VisionComms);
    mockable ~VisionComms();

    mockable void initializeCV();

    void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

    mockable bool isCvOnline() const;

    mockable inline const TurretAimData& getLastAimData(uint8_t turretID) const
    {
        return lastAimData[turretID];
    }

    mockable inline bool isAimDataUpdated(uint8_t turretID) const
    {
        return aimDataUpdated[turretID];
    }

    mockable inline bool getSomeTurretHasTarget() const
    {
        // bool hasTarget = false;
        // for (size_t i = 0; i < control::turret::NUM_TURRETS; i++)
        // {
        //     hasTarget |= lastAimData[i].updated;
        // }
        return false;
    }

    enum MessageType : uint16_t
    {
        TURRET_DATA = 1,
        ROBOT_ID = 2,
        ALIVE = 3
    };

private:
    static constexpr int16_t TIME_OFFLINE_CV_AIM_DATA_MS = 1'000;

    tap::arch::MilliTimeout cvOfflineTimeout;

    TurretAimData lastAimData[control::turret::NUM_TURRETS] = {};

    bool aimDataUpdated[control::turret::NUM_TURRETS] = {};

    mockable void sendRobotIdMessage();

    bool decodeToTurretAimData(const ReceivedSerialMessage& message);
};
}  // namespace src::serial

#endif  // VISION_COMMS_HPP