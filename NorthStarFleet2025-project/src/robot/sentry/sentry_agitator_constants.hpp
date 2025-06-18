#ifndef STANDARD_AGITATOR_CONSTANTS_HPP_
#define STANDARD_AGITATOR_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "control/agitator/unjam_spoke_agitator_command.hpp"
#include "control/agitator/velocity_agitator_subsystem_config.hpp"

// Do not include this file directly: use agitator_constants.hpp instead.
#ifndef AGITATOR_CONSTANTS_HPP_
#error "Do not include this file directly! Use agitator_constants.hpp instead."
#endif

using tap::motor::DjiMotor;

namespace src::control::agitator::constants
{
static constexpr uint16_t HEAT_LIMIT_BUFFER = 25;
// position PID terms
// PID terms for standard
static constexpr tap::algorithms::SmoothPidConfig AGITATOR_PID_CONFIG = {
    .kp = 2'500.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_C610,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
static constexpr int AGITATOR_NUM_POCKETS = 8;         // number of balls in one rotation
static constexpr float AGITATOR_MAX_ROF = 40.0f;       // balls per second
static constexpr float OVERSHOOT_FUDGE_FACTOR = .105;  // how much agitator overshoots

static constexpr src::agitator::VelocityAgitatorSubsystemConfig AGITATOR_CONFIG_BOTTOM = {
    .gearRatio = 1 / 36.0f,
    .agitatorMotorId = tap::motor::MOTOR4,
    .agitatorCanBusId = tap::can::CanBus::CAN_BUS2,
    .isAgitatorInverted = false,
    /**
     * The jamming constants. Agitator is considered jammed if difference between the velocity
     * setpoint and actual velocity is > jammingVelocityDifference for > jammingTime.
     */
    .jammingVelocityDifference = M_TWOPI,
    .jammingTime = 100,
    .jamLogicEnabled = true,
    .velocityPIDFeedForwardGain = 700.0f / M_TWOPI,
};

static constexpr src::agitator::VelocityAgitatorSubsystemConfig AGITATOR_CONFIG_TOP = {
    .gearRatio = 1 / 36.0f,
    .agitatorMotorId = tap::motor::MOTOR3,
    .agitatorCanBusId = tap::can::CanBus::CAN_BUS2,
    .isAgitatorInverted = false,
    /**
     * The jamming constants. Agitator is considered jammed if difference between the velocity
     * setpoint and actual velocity is > jammingVelocityDifference for > jammingTime.
     */
    .jammingVelocityDifference = M_TWOPI,
    .jammingTime = 100,
    .jamLogicEnabled = true,
    .velocityPIDFeedForwardGain = 700.0f / M_TWOPI,
};

static constexpr tap::control::setpoint::MoveIntegralCommand::Config AGITATOR_ROTATE_CONFIG_BOTTOM =
    {
        .targetIntegralChange = M_TWOPI / AGITATOR_NUM_POCKETS - OVERSHOOT_FUDGE_FACTOR,
        .desiredSetpoint = AGITATOR_MAX_ROF * (M_TWOPI / AGITATOR_NUM_POCKETS),
        .integralSetpointTolerance = (M_TWOPI / AGITATOR_NUM_POCKETS) * 0.1f,
};

static constexpr tap::control::setpoint::MoveIntegralCommand::Config AGITATOR_ROTATE_CONFIG_TOP = {
    .targetIntegralChange = M_TWOPI / AGITATOR_NUM_POCKETS - OVERSHOOT_FUDGE_FACTOR,
    .desiredSetpoint = AGITATOR_MAX_ROF * (M_TWOPI / AGITATOR_NUM_POCKETS),
    .integralSetpointTolerance = (M_TWOPI / AGITATOR_NUM_POCKETS) * 0.1f,
};

constexpr float UNJAM_VELOCITY_BOTTOM = 0.35 * AGITATOR_MAX_ROF * (M_TWOPI / AGITATOR_NUM_POCKETS);
constexpr float UNJAM_DISTANCE_BOTTOM = 0.6f * (M_TWOPI / AGITATOR_NUM_POCKETS);
static constexpr src::control::agitator::UnjamSpokeAgitatorCommand::Config
    AGITATOR_UNJAM_CONFIG_BOTTOM = {
        .targetUnjamIntegralChange = UNJAM_DISTANCE_BOTTOM,
        .unjamSetpoint = UNJAM_VELOCITY_BOTTOM,
        /// Unjamming should take unjamDisplacement (radians) / unjamVelocity (radians / second)
        /// seconds.Convert to ms, Add 100 ms extra tolerance.
        .maxWaitTime =
            static_cast<uint32_t>(1000.0f * UNJAM_DISTANCE_BOTTOM / UNJAM_VELOCITY_BOTTOM) + 200,
        .targetCycleCount = 3,
};

constexpr float UNJAM_VELOCITY_TOP = 0.35 * AGITATOR_MAX_ROF * (M_TWOPI / AGITATOR_NUM_POCKETS);
constexpr float UNJAM_DISTANCE_TOP = 0.6f * (M_TWOPI / AGITATOR_NUM_POCKETS);
static constexpr src::control::agitator::UnjamSpokeAgitatorCommand::Config
    AGITATOR_UNJAM_CONFIG_TOP = {
        .targetUnjamIntegralChange = UNJAM_DISTANCE_TOP,
        .unjamSetpoint = UNJAM_VELOCITY_TOP,
        /// Unjamming should take unjamDisplacement (radians) / unjamVelocity (radians / second)
        /// seconds.Convert to ms, Add 100 ms extra tolerance.
        .maxWaitTime =
            static_cast<uint32_t>(1000.0f * UNJAM_DISTANCE_TOP / UNJAM_VELOCITY_TOP) + 200,
        .targetCycleCount = 3,
};

}  // namespace src::control::agitator::constants

#endif  // STANDARD_AGITATOR_CONSTANTS_HPP_