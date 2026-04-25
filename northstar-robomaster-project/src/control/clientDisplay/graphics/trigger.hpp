/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Written by: Alex Stedman of RHIT Thornbots
 */

#ifndef TRIGGER_HPP
#define TRIGGER_HPP

#include <functional>
#include <vector>
#include "tap/drivers.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"
#include "tap/control/command_scheduler.hpp"

namespace tap
{
namespace control
{

enum class MouseButton
{
    LEFT,
    RIGHT
};

enum class MouseScrollDirection
{
    UP,
    DOWN
};

using namespace tap::communication::serial;

/**
 * @class Trigger
 * @brief Represents a trigger condition that can execute commands based on input states.
 */
class Trigger 
{
public:

    /**
     * @brief Constructs a Trigger using a custom condition function.
     * @param drivers Pointer to the Drivers instance.
     * @param condition Function that returns a boolean representing the trigger state.
     */
    Trigger(Drivers* drivers, std::function<bool()> condition);

    /**
     * @brief Constructs a Trigger for a specific remote key.
     * @param drivers Pointer to the Drivers instance.
     * @param key Remote key used as a trigger.
     */
    Trigger(Drivers* drivers, Remote::Key key);

    /**
     * @brief Constructs a Trigger for a specific remote switch and its state.
     * @param drivers Pointer to the Drivers instance.
     * @param sw Remote switch.
     * @param state Desired state of the switch to trigger.
     */
    Trigger(Drivers* drivers, Remote::Switch sw, Remote::SwitchState state);

    /**
     * @brief Constructs a Trigger for a specific remote channel value.
     * @param drivers Pointer to the Drivers instance.
     * @param channel Remote control channel.
     * @param value Threshold value to trigger.
     */
    Trigger(Drivers* drivers, Remote::Channel channel, float value);

    /**
     * @brief Constructs a Trigger for left or right mouse button clicks.
     * @param drivers Pointer to the Drivers instance.
     * @param button Enum indicating left or right click.
     */
    Trigger(Drivers* drivers, MouseButton button);

    /**
     * @brief Constructs a Trigger for up or down mouse scrolling.
     * @param drivers Pointer to the Drivers instance.
     * @param direction Enum for up or down.
     */
    Trigger(Drivers* drivers, MouseScrollDirection direction);

    // Operators to combine triggers
    //and operator
    Trigger operator&(std::function<bool()> trigger);
    //or operator
    Trigger operator|(std::function<bool()> trigger);
    //not operator
    Trigger operator!();
    //xor operator
    Trigger operator^(std::function<bool()> trigger);

    // Converts Trigger to a boolean function
    operator std::function<bool()>() { return m_condition; }

    // Command scheduling methods
    /**
     * @brief Schedules a command to execute when the trigger transitions from true to false.
     * @param command Command to be executed.
     * @return Updated Trigger object.
     */
    Trigger* onFalse(Command* command) { return schedule(command, &m_onFalseCommands); }

    /**
     * @brief Schedules a command to execute when the trigger transitions from false to true.
     * @param command Command to be executed.
     * @return Updated Trigger object.
     */
    Trigger* onTrue(Command* command) { return schedule(command, &m_onTrueCommands); }

    /**
     * @brief Toggles the execution of a command when the trigger toggles from true to false.
     * @param command Command to be toggled.
     * @return Updated Trigger object.
     */
    Trigger* toggleOnFalse(Command* command) { return schedule(command, &m_toggleOnFalseCommands); }

    /**
     * @brief Toggles the execution of a command when the trigger toggles from false to true.
     * @param command Command to be toggled.
     * @return Updated Trigger object.
     */
    Trigger* toggleOnTrue(Command* command) { return schedule(command, &m_toggleOnTrueCommands); }

    /**
     * @brief Executes a command while the trigger is false and cancels it when it is true
     * @param command Command to be executed.
     * @return Updated Trigger object.
     */
    Trigger* whileFalse(Command* command) { return schedule(command, &m_whileFalseCommands); }

    /**
     * @brief Executes a command while the trigger is true and cancels it when it is false
     * @param command Command to be executed.
     * @return Updated Trigger object.
     */
    Trigger* whileTrue(Command* command) { return schedule(command, &m_whileTrueCommands); }

    /**
     * @brief Evaluates the trigger condition and returns its boolean state.
     * @return True if the trigger condition is met, false otherwise.
     */
    bool getAsBoolean() { return m_condition();}// ? m_condition() : false; }

    /**
     * @brief Updates the trigger state and schedules/cancels commands accordingly.
     */
    void update();

private:
    std::function<bool()> m_condition; ///< Function representing the trigger condition.

    // Vectors storing commands associated with trigger states
    std::vector<Command*> m_onFalseCommands;
    std::vector<Command*> m_onTrueCommands;
    std::vector<Command*> m_toggleOnFalseCommands;
    std::vector<Command*> m_toggleOnTrueCommands;
    std::vector<Command*> m_whileFalseCommands;
    std::vector<Command*> m_whileTrueCommands;

    Drivers* m_drivers; ///< Pointer to the Drivers instance.
    bool m_lastValue; ///< Stores the previous trigger state.
    bool m_toggleState; ///< Stores the toggle state for toggle commands.

    /**
     * @brief Schedules a command for a given trigger state.
     * @param command Command to be scheduled.
     * @param commands Vector of commands to store it in.
     * @return Updated Trigger object.
     */
    Trigger* schedule(Command* command, std::vector<Command*>* commands);

    /**
     * @brief Safely schedules commands for execution.
     * @param commands List of commands to execute.
     */
    void safeSchedule(std::vector<Command*> commands);

    /**
     * @brief Safely cancels scheduled commands.
     * @param commands List of commands to cancel.
     */
    void safeCancel(std::vector<Command*> commands);
};

}  // namespace control
}  // namespace tap

#endif  // TRIGGER_HPP
