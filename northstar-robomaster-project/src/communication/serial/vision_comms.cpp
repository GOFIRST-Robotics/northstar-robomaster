#include "vision_comms.hpp"

namespace src::serial
{
VisionComms::VisionComms(
    tap::Drivers* drivers,
    src::chassis::ChassisOdometry* chassisOdometry,
    src::chassis::ChassisSubsystem,
    tap::motor::DjiMotor* pitchMotor)
    : DJISerial(drivers, VISION_COMMS_RX_UART_PORT),
      lastAimData(),
      chassisOdometry(chassisOdometry),
      chassisSubsystem(chassisSubsystem),
      pitchMotor(pitchMotor)
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
        case MessageType::ODOMETRY:
        {
            decodeToOdometeryData(completeMessage);
            return;
        }

        case MessageType::REF_DATA:
        {
            // shouldn't receive this from the CV, only send it
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
    int curreIndex = 0;
    modm::Vector2f receivedPosition = modm::Vector2f();
    float receivedRotation = 0;

    if (sizeof(OdometryData) > message.header.dataLength)
    {
        std::printf("not enough data for odometry");
        return false;  // Not enough data for another turret aim data
    }

    // Position data
    memcpy(&receivedPosition.x, &message.data[curreIndex], sizeof(float));
    curreIndex += sizeof(float);

    memcpy(&receivedPosition.y, &message.data[curreIndex], sizeof(float));
    curreIndex += sizeof(float);

    // rotation Data
    memcpy(&receivedRotation, &message.data[curreIndex], sizeof(float));
    curreIndex += sizeof(float);

    // send the values received back into odometry
    chassisOdometry->setOdometryPosAndRot(receivedPosition, receivedRotation);
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

void VisionComms::sendRobotOdometry()
{
    DJISerial::SerialMessage<sizeof(OdometryData)> odometryMessage;
    OdometryData odometryData = OdometryData();

    modm::Vector2f global_pos = chassisOdometry->getPositionGlobal();
    modm::Vector2f global_vel = chassisOdometry->getVelocityGlobal();

    // Chassis data
    odometryData.chassis_data.pos_x = global_pos.x;  // meters
    odometryData.chassis_data.pos_y = global_pos.y;  // meters
    odometryData.chassis_data.pos_z = 0;  // TODO: this assumes the robot is on level ground.
                                          // Odometry should support z for varied height fields

    odometryData.chassis_data.vel_x = global_vel.x;  // meters/second
    odometryData.chassis_data.vel_y = global_vel.y;  // meters/second
    odometryData.chassis_data.vel_z = 0;             // TODO: see z on position (it doesn't exist)

    // Turret Data
    odometryData.turret_data.turret_pitch = pitchMotor->getPositionWrapped();  // radians
    odometryData.turret_data.turret_yaw = drivers->bmi088.getYaw();            // radians
    odometryData.turret_data.turret_roll = drivers->bmi088.getRoll();          // radians

    odometryMessage.messageType = MessageType::ODOMETRY;

    // Chassis data
    odometryMessage.data[0] = odometryData.chassis_data.pos_x;
    odometryMessage.data[1] = odometryData.chassis_data.pos_y;
    odometryMessage.data[2] = odometryData.chassis_data.pos_z;

    odometryMessage.data[3] = odometryData.chassis_data.vel_x;
    odometryMessage.data[4] = odometryData.chassis_data.vel_y;
    odometryMessage.data[5] = odometryData.chassis_data.vel_z;

    // Turret Data
    odometryMessage.data[6] = odometryData.turret_data.turret_roll;
    odometryMessage.data[7] = odometryData.turret_data.turret_pitch;
    odometryMessage.data[8] = odometryData.turret_data.turret_yaw;

    odometryMessage.setCRC16();
    drivers->uart.write(
        VISION_COMMS_TX_UART_PORT,
        reinterpret_cast<uint8_t*>(&odometryMessage),
        sizeof(odometryMessage));
}

void VisionComms::sendRefData()
{
    DJISerial::SerialMessage<sizeof(RefData)> refDataMessage;



    refDataMessage.messageType = MessageType::REF_DATA;

    // save into the message
    refDataMessage.data[0] = drivers->refSerial.getGameData().airSupportData;


    refDataMessage.setCRC16();
    drivers->uart.write(
        VISION_COMMS_TX_UART_PORT,
        reinterpret_cast<uint8_t*>(&refDataMessage),
        sizeof(refDataMessage));
}

bool VisionComms::isCvOnline() const { return !cvOfflineTimeout.isExpired(); }

}  // namespace src::serial