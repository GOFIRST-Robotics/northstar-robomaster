#ifndef USING_AGITATOR_HPP_
#define USING_AGITATOR_HPP_

#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "control/agitator/fire_rate_cycler_mapping.hpp"
#include "control/cycle_state_command_mapping.hpp"
#include "control/governor/fire_rate_limit_governor.hpp"
#include "control/governor/fired_recently_governor.hpp"
#include "control/governor/plate_hit_governor.hpp"
#include "robot/testbed/test_def.hpp"

using namespace tap::control::setpoint;
using namespace tap::control;
using namespace src::agitator;
using namespace src::control::agitator;
using namespace tap::communication::serial;
using namespace src::control::governor;
using namespace tap::control::governor;

#ifdef USING_AGITATOR

// agitator subsystem
src::agitator::VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG_2);

// agitator commands
ConstantVelocityAgitatorCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

UnjamSpokeAgitatorCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

// agitator governors
ManualFireRateReselectionManager manualFireRateReselectionManager;

SetFireRateCommand setFireRateCommandFullAuto(
    &dummySubsystem,
    manualFireRateReselectionManager,
    40,
    &rotateAgitator);
SetFireRateCommand setFireRateCommand10RPS(
    &dummySubsystem,
    manualFireRateReselectionManager,
    10,
    &rotateAgitator);
SetFireRateCommand setFireRateCommand30RPS(
    &dummySubsystem,
    manualFireRateReselectionManager,
    30,
    &rotateAgitator);

ToggleCommandMapping qPressed10RPS(
    drivers(),
    {&setFireRateCommand10RPS},
    RemoteMapState(RemoteMapState({Remote::Key::Q})));

ToggleCommandMapping wPressed30RPS(
    drivers(),
    {&setFireRateCommand30RPS},
    RemoteMapState(RemoteMapState({Remote::Key::Q})));

ToggleCommandMapping ePressedFullAuto(
    drivers(),
    {&setFireRateCommandFullAuto},
    RemoteMapState(RemoteMapState({Remote::Key::W})));

FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

GovernorLimitedCommand<1> rotateAndUnjamAgitatorLimited(
    {&agitator},
    rotateAndUnjamAgitator,
    {&fireRateLimitGovernor});

HoldRepeatCommandMapping leftMousePressedShoot(
    drivers(),
    {&rotateAndUnjamAgitatorLimited},
    RemoteMapState(RemoteMapState::MouseButton::LEFT),
    false);

HoldRepeatCommandMapping leftSwitchDownPressedShoot(
    drivers(),
    {&rotateAndUnjamAgitatorLimited},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    false);

FireRateCyclerMapping rightSwitchUpPressedShoot(
    *drivers(),
    rotateAndUnjamAgitatorLimited,
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    &manualFireRateReselectionManager,
    &rotateAgitator);

src::control::CycleStateCommandMapping<
    FireRateCyclerMapping::LaunchMode,
    FireRateCyclerMapping::NUM_SHOOTER_STATES,
    FireRateCyclerMapping>
    rightSwitchdownCycleShotSpeed(
        drivers(),
        RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN),
        FireRateCyclerMapping::SINGLE,
        &rightSwitchUpPressedShoot,
        &FireRateCyclerMapping::setShooterState);

#endif

#endif  // USING_AGITATOR_HPP_