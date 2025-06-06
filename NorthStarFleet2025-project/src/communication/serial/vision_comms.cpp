#include "vision_comms.hpp"

namespace src::serial
{
VisionComms::VisionComms(tap::Drivers* drivers)
    : DJISerial(drivers, VISION_COMMS_RX_UART_PORT),
      lastAimData()
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
    drivers->uart.init<VISION_COMMS_RX_UART_PORT, VISION_COMMS_BAUD_RATE>();
}

void VisionComms::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
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

bool VisionComms::decodeToTurretAimData(const ReceivedSerialMessage& message)
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
    DJISerial::SerialMessage<1> robotTypeMessage;
    robotTypeMessage.messageType = MessageType::ROBOT_ID;
    robotTypeMessage.data[0] =
        static_cast<uint8_t>(1);  // drivers->refSerial.getRobotData().robotId);
    robotTypeMessage.setCRC16();
    drivers->uart.write(
        VISION_COMMS_TX_UART_PORT,
        reinterpret_cast<uint8_t*>(&robotTypeMessage),
        sizeof(robotTypeMessage));
}

}  // namespace src::serial