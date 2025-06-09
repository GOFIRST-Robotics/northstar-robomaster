#include "vision_comms.hpp"

namespace src::serial
{
VisionComms::VisionComms(tap::Drivers* drivers) : drivers(drivers), lastAimData()
{
    for (size_t i = 0; i < control::turret::NUM_TURRETS; i++)
    {
        this->lastAimData[i].updated = false;
    }
}

VisionComms::~VisionComms() {}

void VisionComms::initializeCV()
{
    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
    drivers->uart.init<VISION_COMMS_TX_UART_PORT, VISION_COMMS_BAUD_RATE>();
    // drivers->uart.init<VISION_COMMS_RX_UART_PORT, VISION_COMMS_BAUD_RATE>();
}
int countp = 0;
void VisionComms::sendMessage()
{
    if (countp / 1000 >= 1)
    {
        sendRobotIdMessage();
        countp = 0;
    }
    countp++;
}
int readddd = 0;
void VisionComms::read()
{
    tap::communication::serial::DJISerial::SerialMessage<1024> completeMessage;
    uint8_t raw_data[102];
    uint8_t start_idx = 0;
    uint8_t end_idx = 0;
    readddd = drivers->uart.read(VISION_COMMS_RX_UART_PORT, raw_data, sizeof(raw_data));
    for (size_t i = 0; i < sizeof(raw_data); i++)
    {
        if (raw_data[i] == 0xa5)
        {
            if (start_idx == 0)
            {
                start_idx = i;
            }
            else
            {
                end_idx = i;
            }
            if (start_idx && end_idx)
            {
                break;
            }
        }
    }
    if (start_idx && end_idx)
    {
        for (size_t i = start_idx; i < end_idx; i++)
        {
            raw_data[i - start_idx] = raw_data[i];
        }
    }
    else
    {
        return;  // No valid data found
    }
    completeMessage.header.headByte = 0xa5;
    completeMessage.header.dataLength = (raw_data[start_idx + 2] << 8) | raw_data[start_idx + 1];
    completeMessage.header.seq = raw_data[start_idx + 3];
    completeMessage.header.CRC8 = raw_data[start_idx + 4];
    if (!verifyCRC8(
            reinterpret_cast<uint8_t*>(&completeMessage),
            sizeof(completeMessage.header) - 1,
            completeMessage.header.CRC8))
    {
        // CRC8 failed
        return;
    }
    completeMessage.messageType = (raw_data[start_idx + 5] << 8) | raw_data[start_idx + 6];
    for (int i = 0; i < completeMessage.header.dataLength; i++)
    {
        completeMessage.data[i] = raw_data[start_idx + i + 7];
    }
    completeMessage.CRC16 = (raw_data[start_idx + 7 + completeMessage.header.dataLength] << 8) |
                            raw_data[start_idx + 6 + completeMessage.header.dataLength];
    if (completeMessage.CRC16 !=
        tap::algorithms::calculateCRC16(
            reinterpret_cast<uint8_t*>(&completeMessage),
            sizeof(completeMessage.header) + sizeof(completeMessage.messageType) +
                completeMessage.header.dataLength))
    {
        return;
    }

    // if (drivers->uart.read(
    //         VISION_COMMS_RX_UART_PORT,
    //         &completeMessage.data[0],
    //         sizeof(completeMessage.data)) > 0)
    // {
    //     completeMessage.setCRC16();
    // if (completeMessage.isValid())
    // {
    messageReceiveCallback(completeMessage);
    // }
}

void VisionComms::messageReceiveCallback(
    const tap::communication::serial::DJISerial::SerialMessage<1024>& completeMessage)
{
    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);

    switch (completeMessage.messageType)
    {
        case MessageType::TURRET_DATA:
        {
            decodeToTurretAimData(completeMessage);
            return;
        }
        case MessageType::ROBOT_ID:
        {
            sendRobotIdMessage();
            return;
        }
        case 3:
        {
            return;
        }
        case 4:
        {
            return;
        }
        case 5:
        {
            return;
        }

        default:
            break;
    }
}

bool VisionComms::decodeToTurretAimData(
    const tap::communication::serial::DJISerial::SerialMessage<1024>& message)
{
    int curreIndex = 0;
    for (size_t i = 0; i < control::turret::NUM_TURRETS; i++)
    {
        if (curreIndex + sizeof(TurretAimData) > message.header.dataLength)
        {
            return false;  // Not enough data for another turret aim data
        }

        TurretAimData& aimData = lastAimData[i];
        memcpy(&aimData.yaw, &message.data[curreIndex], sizeof(float));
        curreIndex += sizeof(float);

        memcpy(&aimData.pitch, &message.data[curreIndex], sizeof(float));
        curreIndex += sizeof(float);

        aimData.updated = true;
    }
}

void VisionComms::sendRobotIdMessage()
{
    tap::communication::serial::DJISerial::SerialMessage<1> robotTypeMessage;
    robotTypeMessage.messageType = MessageType::ROBOT_ID;
    robotTypeMessage.data[0] =
        static_cast<uint8_t>(0x02);  // drivers->refSerial.getRobotData().robotId);
    robotTypeMessage.setCRC16();
    drivers->uart.write(
        VISION_COMMS_TX_UART_PORT,
        reinterpret_cast<uint8_t*>(&robotTypeMessage),
        sizeof(robotTypeMessage));
}

bool VisionComms::isCvOnline() const { return !cvOfflineTimeout.isExpired(); }

}  // namespace src::serial