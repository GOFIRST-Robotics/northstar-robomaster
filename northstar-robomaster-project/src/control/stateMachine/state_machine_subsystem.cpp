//#define FLY_SKY
#include "state_machine_subsytem.hpp"

namespace src::stateMachine
{
StateMachineSubsystem::StateMachineSubsystem(
    tap::Drivers* drivers,
    src::chassis::ChassisSubsystem* chassisSubsystem,
    src::chassis::ChassisAutoDrive* chassisAutoDrive)
    : Subsystem(drivers),
      drivers(drivers),
      chassisSubsystem(chassisSubsystem),
      chassisAutoDrive(chassisAutoDrive)
{
}

void StateMachineSubsystem::initialize() {}

bool beyblade = false;
bool a = false;
float t = 0;

void StateMachineSubsystem::refresh()
{
    // return;
#ifdef FLY_SKY
    if (drivers->remote.getChannel(5))
    {
#else
    if (drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) !=
        tap::communication::serial::Remote::SwitchState::UP)
    {
#endif

        return;
    }
    if (t > 1)
    {
        chassisSubsystem->setVelocityFieldDrive(0, 0, 0);
        return;
    }
    if (!a)
    {
        a = true;
        chassisAutoDrive->addCurveToPath(CubicBezier(
            modm::Vector<float, 2>(0, 0),
            modm::Vector<float, 2>(-0.4, 4.15),
            modm::Vector<float, 2>(-0.4, 1),
            modm::Vector<float, 2>(-0.4, 3.15)));

        chassisAutoDrive->addCurveToPath(CubicBezier(
            modm::Vector<float, 2>(-0.4, 4.15),
            modm::Vector<float, 2>(-2.18, 4.15),
            modm::Vector<float, 2>(-0.4, 2.7),
            modm::Vector<float, 2>(-2.18, 2.7)));

        chassisAutoDrive->addCurveToPath(CubicBezier(
            modm::Vector<float, 2>(-2.18, 4.15),
            modm::Vector<float, 2>(-0.4, 4.15),
            modm::Vector<float, 2>(-2.18, 2.7),
            modm::Vector<float, 2>(-0.4, 2.7)));
    }

    chassisAutoDrive->updateAutoDrive();
    modm::Vector<float, 2> desiredGlobalVelocity = chassisAutoDrive->getDesiredGlobalVelocity();
    float desiredRadiansPerSecond = 0.0f;
    if (beyblade)
    {
        desiredRadiansPerSecond = chassisSubsystem->calculateMaxRotationSpeed(
            desiredGlobalVelocity.y,
            desiredGlobalVelocity.x);
    }
    else
    {
        desiredRadiansPerSecond =
            chassisAutoDrive
                ->getDesiredRotation();  // use chassisSpeedRotationPID with heading passed in
    }

    chassisSubsystem->setVelocityFieldDrive(
        desiredGlobalVelocity.y,
        desiredGlobalVelocity.x,
        desiredRadiansPerSecond);
}  // namespace src::stateMachine

}  // namespace src::stateMachine