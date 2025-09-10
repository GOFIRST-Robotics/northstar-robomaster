#include "vision_comms.hpp"

namespace src::serial
{
VisionComms::VisionComms(tap::Drivers* drivers)
    : DJISerial(drivers, VISION_COMMS_RX_UART_PORT),
      lastAimData()
{
    for (size_t i = 0; i < control::turret::NUM_TURRETS; i++)
    {
        // this->lastAimData[i].updated = false;
    }
}

VisionComms::~VisionComms() {}

void VisionComms::initializeCV()
{
    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
    drivers->uart.init<VISION_COMMS_TX_UART_PORT, VISION_COMMS_BAUD_RATE>();
}

void VisionComms::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
{
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
        case MessageType::ALIVE:
        {
            cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
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
        if ((curreIndex + sizeof(TurretAimData) - sizeof(float) * 2 - 2) >
            message.header.dataLength)
        {
            return false;  // Not enough data for another turret aim data
        }

        TurretAimData& aimData = lastAimData[i];
        memcpy(&aimData.yaw, &message.data[curreIndex], sizeof(float));
        curreIndex += sizeof(float);

        memcpy(&aimData.pitch, &message.data[curreIndex], sizeof(float));
        curreIndex += sizeof(float);

        if (aimData.yaw == 0 && aimData.pitch == 0)
        {
            aimDataUpdated[i] = false;
        }
        else
        {
            aimDataUpdated[i] = true;
        }

        memcpy(&aimData.distance, &message.data[curreIndex], sizeof(float));
        curreIndex += sizeof(float);

        memcpy(&aimData.robotId, &message.data[curreIndex], sizeof(aimData.robotId));
        curreIndex += sizeof(aimData.robotId);

        if (aimData.distance != 0)
        {
            auto it = plateLookup.find(uint8_t(aimData.robotId));
            if (it != plateLookup.end())
            {
                PlateDims dims = it->second;
                aimData.maxErrorYaw = atan((dims.width / 2) / aimData.distance);
                aimData.maxErrorPitch = atan((dims.height / 2) / aimData.distance);
            }
        }
    }
}

void VisionComms::sendRobotIdMessage()
{
    DJISerial::SerialMessage<1> robotTypeMessage;
    robotTypeMessage.messageType = MessageType::ROBOT_ID;
    robotTypeMessage.data[0] = static_cast<uint8_t>(drivers->refSerial.getRobotData().robotId);
    robotTypeMessage.setCRC16();
    drivers->uart.write(
        VISION_COMMS_TX_UART_PORT,
        reinterpret_cast<uint8_t*>(&robotTypeMessage),
        sizeof(robotTypeMessage));
}

bool VisionComms::isCvOnline() const { return !cvOfflineTimeout.isExpired(); }

}  // namespace src::serial