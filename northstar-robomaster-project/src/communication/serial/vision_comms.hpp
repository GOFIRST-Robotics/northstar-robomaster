#ifndef VISION_COMMS_HPP
#define VISION_COMMS_HPP

#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/drivers.hpp"

#include "control/turret/constants/turret_constants.hpp"
#include "control/chassis/chassis_odometry.hpp"
#include "control/chassis/chassis_subsystem.hpp"

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

    src::chassis::ChassisOdometry* chassisOdometry;

    src::chassis::ChassisSubsystem* chassisSubsystem;

    tap::motor::DjiMotor* pitchMotor;

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
        {1, {.2f, .15f}},   // hero plate dimentions in mm
        {7, {.15f, .15f}},  // sentry plate dimentions in mm
        {3, {.15f, .15f}},  // don't know id 3 is correct
    };

    VisionComms(tap::Drivers* drivers, src::chassis::ChassisOdometry* chassisOdometry, src::chassis::ChassisSubsystem chassisSubsystem, tap::motor::DjiMotor* yawMotor);
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
        ALIVE = 3,
        ODOMETRY = 4,
        REF_DATA = 5
    };

    
    struct RefData{

        uint8_t game_status;
        uint8_t game_result;
        float all_robot_hp[6];

        uint8_t site_event_data;
        uint8_t warning_data;
        uint8_t dart_info;

        uint8_t robot_status;
        uint16_t power_and_heat;
        float robot_position[2];
        uint8_t robot_buff_status;
        uint8_t receive_damage;
        uint8_t projectile_launch;
        uint16_t bullets_remain;
        uint8_t rfid_status;
        uint8_t dart_station_info[4];
        float ground_robot_position[2];
        uint8_t radar_progress;
        uint8_t sentry_info;
        uint8_t radar_info;
        //custom_data;

        
    };
    

    struct TurretOdometryData
    {
        float turret_pitch;
        float turret_yaw;
        float turret_roll;

    }modm_packed;

    struct ChassisOdometryData
    {
        float pos_x;
        float pos_y;
        float pos_z;

        float vel_x;
        float vel_y;
        float vel_z;

    }modm_packed;

    struct OdometryData
    {
        TurretOdometryData turret_data;
        ChassisOdometryData chassis_data;

    }modm_packed;

private:
    static constexpr int16_t TIME_OFFLINE_CV_AIM_DATA_MS = 1'000;

    tap::arch::MilliTimeout cvOfflineTimeout;

    TurretAimData lastAimData[control::turret::NUM_TURRETS] = {};

    bool aimDataUpdated[control::turret::NUM_TURRETS] = {};

    mockable void sendRobotIdMessage();

    mockable void sendRobotOdometry();

    bool decodeToTurretAimData(const ReceivedSerialMessage& message);

    bool decodeToOdometeryData(const ReceivedSerialMessage& message);

};
}  // namespace src::serial

#endif  // VISION_COMMS_HPP