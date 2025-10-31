#include "vision_comms.hpp"

namespace src::serial
{
VisionComms::VisionComms(tap::Drivers* drivers,src::chassis::ChassisOdometry* chassisOdometry)
    : DJISerial(drivers, VISION_COMMS_RX_UART_PORT),
      lastAimData(),
      chassisOdometry(chassisOdometry)
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


void VisionComms::sendRobotOdometry()
{

    DJISerial::SerialMessage<sizeof(OdometryData)> odometryMessage;
    OdometryData odometryData = OdometryData();
    

    modm::Vector2f global_pos = chassisOdometry->getPositionGlobal();
    float rot = chassisOdometry->getRotation();
    modm::Vector2f global_vel = chassisOdometry->getVelocityGlobal();

    //Chassis data
    odometryData.chassis_data.pos_x = global_pos.x;
    odometryData.chassis_data.pos_y = global_pos.y;
    odometryData.chassis_data.pos_z = 0; //TODO: this assumes the robot is on level ground. Odometry should support z for varied height fields

    odometryData.chassis_data.rot_r = 0;//TODO: these assume that there is no rotation in the other axis, odometry should support these for slopes
    odometryData.chassis_data.rot_p = 0;
    odometryData.chassis_data.rot_y = rot;

    odometryData.chassis_data.vel_x = global_vel.x;
    odometryData.chassis_data.vel_y = global_vel.y;
    odometryData.chassis_data.vel_z = 0;//TODO: see z on position (it doesn't exist)


    //Turret Data
    odometryData.turret_data.turret_roll = 10;
    odometryData.turret_data.turret_yaw = 11;



    odometryMessage.messageType = MessageType::ODOMETRY;
    
    

    //Chassis data
    odometryMessage.data[0] = odometryData.chassis_data.pos_x;
    odometryMessage.data[1] = odometryData.chassis_data.pos_y;
    odometryMessage.data[2] = odometryData.chassis_data.pos_z;

    odometryMessage.data[3] = odometryData.chassis_data.rot_r;
    odometryMessage.data[4] = odometryData.chassis_data.rot_p;
    odometryMessage.data[5] = odometryData.chassis_data.rot_y;

    odometryMessage.data[6] = odometryData.chassis_data.vel_x;
    odometryMessage.data[7] = odometryData.chassis_data.vel_y;
    odometryMessage.data[8] = odometryData.chassis_data.vel_z;


    //Turret Data
    odometryMessage.data[9] = odometryData.turret_data.turret_roll;
    odometryMessage.data[10] = odometryData.turret_data.turret_yaw;




    odometryMessage.setCRC16();
    drivers->uart.write(
        VISION_COMMS_TX_UART_PORT,
        reinterpret_cast<uint8_t*>(&odometryMessage),
        sizeof(odometryMessage));
}

bool VisionComms::isCvOnline() const { return !cvOfflineTimeout.isExpired(); }

}  // namespace src::serial