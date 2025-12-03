#include "vision_comms.hpp"

namespace src::serial
{
VisionComms::VisionComms(tap::Drivers* drivers)
    : DJISerial(drivers, VISION_COMMS_RX_UART_PORT),
      lastAimData(),
      chassisOdometry(nullptr),
      chassisAutoDrive(nullptr),
      pitchMotor(nullptr)
{
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
        case MessageType::ODOMETRY:
        {
            decodeToOdometeryData(completeMessage);
            return;
        }

        case MessageType::AUTO_PATH:
        {
            decodeToAutoPathData(completeMessage);
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

bool VisionComms::decodeToOdometeryData(const ReceivedSerialMessage& message)
{
    if (sizeof(OdometryData) > message.header.dataLength)
    {
        return false;
    }

    OdometryData cleanData;

    std::memcpy(&cleanData, message.data, sizeof(OdometryData));

    chassisOdometry->setOdometry(
        {cleanData.chassis_data.pos_x, cleanData.chassis_data.pos_y},
        {cleanData.chassis_data.vel_x, cleanData.chassis_data.vel_y},
        cleanData.turret_data.turret_yaw);

    return true;
}

bool VisionComms::decodeToAutoPathData(const ReceivedSerialMessage& message)
{
    assert(chassisAutoDrive != nullptr);

    if (sizeof(CubicBezier::CurveData) > message.header.dataLength)
    {
        return false;
    }

    CubicBezier::CurveData wireData;

    std::memcpy(&wireData, message.data, sizeof(CubicBezier::CurveData));

    CubicBezier newCurve(wireData);

    chassisAutoDrive->resetPath();
    chassisAutoDrive->addCurveToPath(newCurve);

    return true;
}

void VisionComms::sendMessage() { sendRobotOdometry(); }

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

void VisionComms::sendRobotOdometry()
{
    if (sendOdometryMsgTimeout.execute())
    {
        assert(chassisOdometry != nullptr);
        assert(pitchMotor != nullptr);

        DJISerial::SerialMessage<sizeof(OdometryData)> odometryMessage;

        odometryMessage.messageType = MessageType::ODOMETRY;

        OdometryData* data = reinterpret_cast<OdometryData*>(odometryMessage.data);

        modm::Vector2f global_pos = chassisOdometry->getPositionGlobal();
        modm::Vector2f global_vel = chassisOdometry->getVelocityGlobal();

        // Chassis data
        data->chassis_data.pos_x = global_pos.x;  // meters
        data->chassis_data.pos_y = global_pos.y;  // meters
        data->chassis_data.pos_z = 0;  // TODO: this assumes the robot is on level ground.
                                       // Odometry should support z for varied height fields

        data->chassis_data.vel_x = global_vel.x;  // meters/second
        data->chassis_data.vel_y = global_vel.y;  // meters/second
        data->chassis_data.vel_z = 0;             // TODO: see z on position (it doesn't exist)

        // Turret Data
        data->turret_data.turret_pitch = pitchMotor->getPositionWrapped();  // radians
        data->turret_data.turret_yaw = drivers->bmi088.getYaw();            // radians
        data->turret_data.turret_roll = drivers->bmi088.getRoll();          // radians

        odometryMessage.setCRC16();
        drivers->uart.write(
            VISION_COMMS_TX_UART_PORT,
            reinterpret_cast<uint8_t*>(&odometryMessage),
            sizeof(odometryMessage));
    }
}

void VisionComms::sendRefData()
{
    DJISerial::SerialMessage<sizeof(RefData)> refDataMessage;

    refDataMessage.messageType = MessageType::REF_DATA;

    // save into the message
    refDataMessage.data[0] = static_cast<uint16_t>(drivers->refSerial.getRobotData().robotId);

    refDataMessage.setCRC16();
    drivers->uart.write(
        VISION_COMMS_TX_UART_PORT,
        reinterpret_cast<uint8_t*>(&refDataMessage),
        sizeof(refDataMessage));
}

// Should do something like this (from ARUW)

// void VisionCoprocessor::sendRobotTypeData()
// {
//     if (sendRobotIdTimeout.execute())
//     {
//         DJISerial::SerialMessage<1> robotTypeMessage;
//         robotTypeMessage.messageType = CV_MESSAGE_TYPE_ROBOT_ID;
//         robotTypeMessage.data[0] =
//         static_cast<uint8_t>(drivers->refSerial.getRobotData().robotId);
//         robotTypeMessage.setCRC16();
//         drivers->uart.write(
//             VISION_COPROCESSOR_TX_UART_PORT,
//             reinterpret_cast<uint8_t*>(&robotTypeMessage),
//             sizeof(robotTypeMessage));
//     }
// }

bool VisionComms::isCvOnline() const { return !cvOfflineTimeout.isExpired(); }

}  // namespace src::serial