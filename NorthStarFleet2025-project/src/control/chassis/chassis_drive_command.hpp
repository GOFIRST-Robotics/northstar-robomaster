#pragma once

#include "tap/control/command.hpp"

namespace control
{
class ControlOperatorInterface;
}

namespace control::chassis
{
class ChassisSubsystem;

/**
 * @brief Commands a ChassisSubsystem using tank drive. The left and right sides of the chassis are
 * commanded independently by this command. This command executes constantly, taking remote inputs
 * in from a control operator interface, transforming remote input into chassis speed.
 */
class ChassisDriveCommand : public tap::control::Command
{
public:
    static constexpr float MAX_CHASSIS_SPEED_MPS = 3.0f;

    /**
     * @brief Construct a new Chassis Tank Drive Command object
     *
     * @param chassis Chassis to control.
     */
    ChassisDriveCommand(ChassisSubsystem &chassis, ControlOperatorInterface &operatorInterface);

    const char *getName() const override { return "Chassis tank drive"; }

    void initialize() override {}

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    ChassisSubsystem &chassis;

    ControlOperatorInterface &operatorInterface;
};
}  // namespace control::chassis