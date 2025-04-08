#pragma once

#include "tap/control/command.hpp"

namespace src
{
class Drivers;

namespace control
{
class ControlOperatorInterface;
}
}  //

namespace src::chassis
{
class ChassisSubsystem;

/**
 * @brief Commands a ChassisSubsystem using tank drive. The left and right sides of the chassis are
 * commanded independently by this command. This command executes constantly, taking remote inputs
 * in from a control operator interface, transforming remote input into chassis speed.
 */
class ChassisBeybladeCommand : public tap::control::Command
{
public:
    static constexpr float MAX_CHASSIS_SPEED_MPS = 3.0f;

    /**
     * @brief Construct a new Chassis Tank Drive Command object
     *
     * @param chassis Chassis to control.
     */
    ChassisBeybladeCommand(ChassisSubsystem *chassis, src::control::ControlOperatorInterface* operatorInterface);

    const char *getName() const override { return "Chassis tank drive"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    src::chassis::ChassisSubsystem *chassis;

    src::control::ControlOperatorInterface* operatorInterface;

    uint32_t prevTime; 
};
}  // namespace control::chassis