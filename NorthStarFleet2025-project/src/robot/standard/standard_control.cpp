#ifdef TARGET_STANDARD

#include "tap/drivers.hpp"
#include "drivers_singleton.hpp"
#include "../../robot-type/robot_type.hpp"

#include "tap/util_macros.hpp"
#include "control/turret/turret_constants/standard_turret_constants.hpp"
#include "robot/standard/standard_drivers.hpp"

#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/chassis_drive_command.hpp"

#include "control/turret/turret_super_structure/standard_turret_subsystem.hpp"
#include "control/turret/turret_control/turret_user_control_command.hpp"
#include "control/turret/turret_components/chassis_frame_turret_controller.hpp"
#include "control/turret/turret_components/yaw_turret_motor.hpp"

#include "control/chassis/constants/chassis_constants.hpp"



using tap::can::CanBus;
using tap::communication::serial::Remote;
// using tap::control::RemoteMapState;
using tap::motor::MotorId;
using namespace control;
using namespace control::chassis;
using namespace src::standard;
using namespace control::turret;
using namespace control::turret::user;
using namespace control::turret::algorithms;
using namespace src::chassis;
// using tap::control::setpoint::IntegrableSetpointSubsystem;
// using tap::control::setpoint::MoveIntegralCommand;
// using tap::control::setpoint::UnjamIntegralCommand;
// using tap::control::setpoint::MoveUnjamIntegralComprisedCommand;

using namespace control::turret;

driversFunc drivers = DoNotUse_getDrivers;

namespace standard_control
{
    ChassisSubsystem chassisSubsystem(
                  drivers(),
                  control::chassis::ChassisConfig{
                    .leftFrontId = LEFT_FRONT_MOTOR_ID,
                    .leftBackId = LEFT_BACK_MOTOR_ID,
                    .rightBackId = RIGHT_BACK_MOTOR_ID,
                    .rightFrontId = RIGHT_FRONT_MOTOR_ID,
                    .canBus = CanBus::CAN_BUS1,
                    .wheelVelocityPidConfig = modm::Pid<float>::Parameter(VELOCITY_PID_KP,
                                                                          VELOCITY_PID_KI,
                                                                          VELOCITY_PID_KD,
                                                                          VELOCITY_PID_MAX_ERROR_SUM),
                  },
                  &drivers()->turretMCBCanCommBus1);

    ChassisDriveCommand chassisDriveCommand(
        &chassisSubsystem,
        &drivers()->controlOperatorInterface);
    //   agitatorSubsystemConfig{
    //     .gearRatio = 36.0f,
    //     .agitatorMotorId = tap::motor::MOTOR7,
    //     .agitatorCanBusId = tap::can::CanBus::CAN_BUS1,
    //     .isAgitatorInverted = false,
    //     .jammingVelocityDifference = M_TWOPI,
    //     .jammingTime = 100,
    //     .jamLogicEnabled = true,
    //     .velocityPIDFeedForwardGain = 500.0f / M_TWOPI
    // },
    // agitatorVelocityPidConfig {
    // .kp = 5'000.0f,
    // .ki = 0.0f,
    // .kd = 0.0f,
    // .maxICumulative = 0.0f,
    // .maxOutput = 10000, //DjiMotor::MAX_OUTPUT_C610,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
    // },
    // agitatorSubsystem(
    //     &drivers, 
    //     agitatorVelocityPidConfig, 
    //     agitatorSubsystemConfig),
    // rotateAgitatorCommandConfig{
    //     .targetIntegralChange = M_TWOPI / 10.0f,
    //     .desiredSetpoint = M_TWOPI,
    //     .integralSetpointTolerance = 0,
    // },
    // rotateAgitatorCommand(
    //     agitatorSubsystem,
    //     rotateAgitatorCommandConfig),
    // unjamAgitatorCommandConfig{
    //     .targetUnjamIntegralChange = 0.6f * (M_TWOPI / 8), //AGITATOR_NUM_POCKETS),
    //     .unjamSetpoint = 0.15f * 20 * (M_TWOPI / 8),
    //     /// Unjamming should take unjamDisplacement (radians) / unjamVelocity (radians / second)
    //     /// seconds.Convert to ms, Add 100 ms extra tolerance.
    //     .maxWaitTime = static_cast<uint32_t>(
    //                    1000.0f * (M_TWOPI / 8) / 0.2f * 20 *
    //                    (M_TWOPI / 8)) +
    //                100,
    //     .targetCycleCount = 3,
    // },
    // unjamAgitatorCommand(
    //     agitatorSubsystem,
    //     unjamAgitatorCommandConfig),
    // rotateAndUnjamAgitatorCommand(
    //     drivers, 
    //     agitatorSubsystem,
    //     rotateAgitatorCommand,
    //     unjamAgitatorCommand),
    // leftMousePressed(
    //     &drivers,
    //     {&rotateAndUnjamAgitatorCommand},
    //     RemoteMapState(
    //         RemoteMapState::MouseButton::LEFT)),
    // rightMousePressed(
    //     &drivers,
    //     {&rotateAndUnjamAgitatorCommand},
    //     RemoteMapState(
    //         RemoteMapState::MouseButton::RIGHT),
    //         true),
    //             m_FlyWheel(
    //     drivers,
    //     &drivers.pwm,
    //     tap::gpio::Pwm::C6,
    //     tap::gpio::Pwm::C7),
    // m_ControlOperatorInterface(drivers),
    // m_FlyWheelCommand(m_FlyWheel, m_ControlOperatorInterface),
    //     leftSwitchUp(
    //     &drivers,
    //     {&rotateAndUnjamAgitatorCommand},
    //     RemoteMapState(
    //         Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP), 
    //         true ),
    tap::motor::DjiMotor pitchMotor(
        drivers(), 
        MotorId::MOTOR5, 
        CanBus::CAN_BUS1, 
        true, "pitchMotor");

    TurretMotor turretPitchMotor(
        &pitchMotor,
        PITCH_MOTOR_CONFIG);
    
    tap::motor::DjiMotor yawMotor(
        drivers(), 
        MotorId::MOTOR8, 
        CanBus::CAN_BUS1, 
        false, "YawMotor");
        
    TurretMCBCGryo turretGyro(
        drivers());

    YawTurretMotor turretYawMotor(
        &yawMotor,
        YAW_MOTOR_CONFIG,
        &turretGyro);

    StandardTurretSubsystem turret(
        drivers(),
        &pitchMotor,
        &yawMotor, 
        PITCH_MOTOR_CONFIG,
        YAW_MOTOR_CONFIG, 
        turretGyro);

    ChassisFrameYawTurretController yawController(
        turretYawMotor, 
        YAW_PID_CONFIG);

    ChassisFramePitchTurretController pitchController(
        turretPitchMotor, 
        PITCH_PID_CONFIG);

    TurretUserControlCommand turretUserControlCommand(
        drivers(),
        &drivers()->controlOperatorInterface,
        &drivers()->turretMCBCanCommBus1,
        &turret, 
        &yawController,
        &pitchController,
        USER_YAW_INPUT_SCALAR,
        USER_PITCH_INPUT_SCALAR,
        0);
    //     turretOrientedDriveCommand(
    //     &drivers,
    //     m_ControlOperatorInterface,
    //     &m_ChassisSubsystem,
    //     &turretYawMotor
    // )
    
void initializeSubsystems(Drivers *drivers)
{
    chassisSubsystem.initialize();
    // agitatorSubsystem.initialize();
    // m_FlyWheel.initialize();
    turret.initialize();
}

void registerSoldierSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassisSubsystem);
    // drivers.commandScheduler.registerSubsystem(&agitatorSubsystem);
    // drivers.commandScheduler.registerSubsystem(&m_FlyWheel);
    drivers->commandScheduler.registerSubsystem(&turret);
}

void setDefaultSoldierCommands(Drivers *drivers)
{
    chassisSubsystem.setDefaultCommand(&chassisDriveCommand);
    // m_FlyWheel.setDefaultCommand(&m_FlyWheelCommand);
    turret.setDefaultCommand(&turretUserControlCommand);
}

void startSoldierCommands(Drivers *drivers) {}

void registerSoldierIoMappings(Drivers *drivers)
{
    // drivers.commandMapper.addMap(&leftMousePressed);
    // drivers.commandMapper.addMap(&rightMousePressed);
    // drivers.commandMapper.addMap(&leftSwitchUp);

}
}  // namespace standard_control

namespace src::standard
{
void initSubsystemCommands(src::standard::Drivers *drivers)
{
    standard_control::initializeSubsystems(drivers);
    standard_control::registerSoldierSubsystems(drivers);
    standard_control::setDefaultSoldierCommands(drivers);
    standard_control::startSoldierCommands(drivers);
    standard_control::registerSoldierIoMappings(drivers);
}
} //namespace src::standard

#endif