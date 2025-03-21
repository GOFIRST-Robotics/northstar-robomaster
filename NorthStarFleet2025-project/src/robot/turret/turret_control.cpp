#ifdef TURRET

#include "tap/drivers.hpp"
#include "drivers_singleton.hpp"
#include "../../robot-type/robot_type.hpp"

#include "tap/util_macros.hpp"
#include "control/turret/turret_constants/standard_turret_constants.hpp"
#include "robot/turret/turret_drivers.hpp"

#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/chassis_drive_command.hpp"

#include "control/turret/turret_super_structure/standard_turret_subsystem.hpp"
#include "control/turret/turret_control/turret_user_control_command.hpp"
#include "control/turret/turret_components/chassis_frame_turret_controller.hpp"
#include "control/turret/turret_components/yaw_turret_motor.hpp"



using tap::can::CanBus;
using tap::communication::serial::Remote;
// using tap::control::RemoteMapState;
using tap::motor::MotorId;
using namespace control;
using namespace src::turret;
using namespace control::turret;
using namespace control::turret::user;
using namespace control::turret::algorithms;
// using tap::control::setpoint::IntegrableSetpointSubsystem;
// using tap::control::setpoint::MoveIntegralCommand;
// using tap::control::setpoint::UnjamIntegralCommand;
// using tap::control::setpoint::MoveUnjamIntegralComprisedCommand;

using namespace control::turret;

driversFunc drivers = DoNotUse_getDrivers;

namespace turret_control
{
    
    
void initializeSubsystems(Drivers *drivers)
{
    
}

void registerSoldierSubsystems(Drivers *drivers)
{
   
}

void setDefaultSoldierCommands(Drivers *drivers)
{
    
}

void startSoldierCommands(Drivers *drivers) {}

void registerSoldierIoMappings(Drivers *drivers)
{
   
}
}  // namespace turret_control

namespace src::turret
{
void initSubsystemCommands(src::turret::Drivers *drivers)
{
    turret_control::initializeSubsystems(drivers);
    turret_control::registerSoldierSubsystems(drivers);
    turret_control::setDefaultSoldierCommands(drivers);
    turret_control::startSoldierCommands(drivers);
    turret_control::registerSoldierIoMappings(drivers);
}
} //namespace src::turret

#endif