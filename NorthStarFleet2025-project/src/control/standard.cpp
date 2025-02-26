#include "drivers.hpp"
#include "../../robot-type/robot_type.hpp"

#include "standard.hpp"

#include "tap/util_macros.hpp"
#include "turret/turret_constants/standard_turret_constants.hpp"


using tap::can::CanBus;
using tap::communication::serial::Remote;
using tap::control::RemoteMapState;
using tap::motor::MotorId;
// using tap::control::setpoint::IntegrableSetpointSubsystem;
// using tap::control::setpoint::MoveIntegralCommand;
// using tap::control::setpoint::UnjamIntegralCommand;
// using tap::control::setpoint::MoveUnjamIntegralComprisedCommand;

using namespace control::turret;

namespace control
{
Robot::Robot(src::Drivers &drivers) 
    : drivers(drivers),
      chassisSubsystem(
          drivers,
          chassis::ChassisConfig{
              .leftFrontId = MotorId::MOTOR2,
              .leftBackId = MotorId::MOTOR3,
              .rightBackId = MotorId::MOTOR4,
              .rightFrontId = MotorId::MOTOR1,
              .canBus = CanBus::CAN_BUS1,
              .wheelVelocityPidConfig = modm::Pid<float>::Parameter(14, 0, 0, 0, 16'000),
          }),
          chassisDriveCommand(chassisSubsystem, m_ControlOperatorInterface),
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
    m_ControlOperatorInterface(drivers.remote),
    // m_FlyWheelCommand(m_FlyWheel, m_ControlOperatorInterface),
    //     leftSwitchUp(
    //     &drivers,
    //     {&rotateAndUnjamAgitatorCommand},
    //     RemoteMapState(
    //         Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP), 
    //         true ),
    pitchMotor(
        &drivers, 
        MotorId::MOTOR5, 
        CanBus::CAN_BUS1, 
        true, "pitchMotor"),
    turretPitchMotor(
        &pitchMotor,
        PITCH_MOTOR_CONFIG),
    yawMotor(
        &drivers, 
        MotorId::MOTOR8, 
        CanBus::CAN_BUS1, 
        false, "YawMotor"),
    turretGyro(
        &drivers),
    turretYawMotor(
        &yawMotor,
        YAW_MOTOR_CONFIG,
        &turretGyro
    ),
    turret(
        &drivers,
        &pitchMotor,
        &yawMotor, 
        PITCH_MOTOR_CONFIG,
        YAW_MOTOR_CONFIG, 
        turretGyro),
    yawController(
        turretYawMotor, 
        YAW_PID_CONFIG),
    pitchController(
        turretPitchMotor, 
        PITCH_PID_CONFIG),
    turretUserControlCommand(
        &drivers,
        m_ControlOperatorInterface,
        &turret, 
        &yawController,
        &pitchController,
        USER_YAW_INPUT_SCALAR,
        USER_PITCH_INPUT_SCALAR,
        0
    )
    //     turretOrientedDriveCommand(
    //     &drivers,
    //     m_ControlOperatorInterface,
    //     &m_ChassisSubsystem,
    //     &turretYawMotor
    // )
{ 
    
}

void Robot::initSubsystemCommands()
{
    initializeSubsystems();
    registerSoldierSubsystems();
    setDefaultSoldierCommands();
    startSoldierCommands();
    registerSoldierIoMappings();
}

void Robot::initializeSubsystems()
{
    chassisSubsystem.initialize();
    // agitatorSubsystem.initialize();
    // m_FlyWheel.initialize();
    turret.initialize();
}

void Robot::registerSoldierSubsystems()
{
    drivers.commandScheduler.registerSubsystem(&chassisSubsystem);
    // drivers.commandScheduler.registerSubsystem(&agitatorSubsystem);
    // drivers.commandScheduler.registerSubsystem(&m_FlyWheel);
    drivers.commandScheduler.registerSubsystem(&turret);
}

void Robot::setDefaultSoldierCommands()
{
    chassisSubsystem.setDefaultCommand(&chassisDriveCommand);
    // m_FlyWheel.setDefaultCommand(&m_FlyWheelCommand);
    turret.setDefaultCommand(&turretUserControlCommand);
}

void Robot::startSoldierCommands() {}

void Robot::registerSoldierIoMappings()
{
    // drivers.commandMapper.addMap(&leftMousePressed);
    // drivers.commandMapper.addMap(&rightMousePressed);
    // drivers.commandMapper.addMap(&leftSwitchUp);

}   
}  // namespace control