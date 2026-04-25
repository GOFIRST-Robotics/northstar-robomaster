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

#include "trigger.hpp"

namespace tap
{
namespace control
{

Trigger::Trigger(Drivers* drivers, std::function<bool()> condition)
    : m_condition(condition),
      m_drivers(drivers),
      m_lastValue(false),
      m_toggleState(false)
{
}

Trigger::Trigger(Drivers* drivers, Remote::Key key)
    : Trigger(drivers, [drivers, key]() { return drivers->remote.keyPressed(key); })
{
}

Trigger::Trigger(Drivers* drivers, Remote::Switch sw, Remote::SwitchState state)
    : Trigger(drivers, [drivers, sw, state]() { return drivers->remote.getSwitch(sw) == state; })
{
}

Trigger::Trigger(Drivers* drivers, Remote::Channel channel, float value)
    : m_drivers(drivers),
      m_lastValue(false),
      m_toggleState(false)
{
    if(value > 0) m_condition = [drivers, channel, value]() { return drivers->remote.getChannel(channel) > value; };
    else m_condition = [drivers, channel, value]() { return drivers->remote.getChannel(channel) < value; };
}

Trigger::Trigger(Drivers* drivers, MouseButton mouseButton)
    : m_drivers(drivers),
      m_lastValue(false),
      m_toggleState(false)
{
    if(mouseButton == MouseButton::LEFT) m_condition =  [drivers]() { return drivers->remote.getMouseL(); };
    else m_condition = [drivers]() { return drivers->remote.getMouseR(); };

}
Trigger::Trigger(Drivers* drivers, MouseScrollDirection direction)
    : m_drivers(drivers),
      m_lastValue(false),
      m_toggleState(false)
{
    if(direction == MouseScrollDirection::UP) m_condition =  [drivers]() { return drivers->remote.getMouseZ()>0; };
    else m_condition = [drivers]() { return drivers->remote.getMouseZ()<0; };

}
Trigger Trigger::operator&(std::function<bool()> trigger)
{
    auto triggerCpy = m_condition; //done to lower context switching. Allows dealloc of triggers made in the lambda
    return Trigger(m_drivers, [triggerCpy, trigger]() { return triggerCpy() && trigger(); });
}

Trigger Trigger::operator!()
{   
    auto triggerCpy = m_condition; //done to lower context switching. Allows dealloc of triggers made in the lambda
    return Trigger(m_drivers, [triggerCpy]() { return !triggerCpy(); });
}

Trigger Trigger::operator|(std::function<bool()> trigger)
{   
    auto triggerCpy = m_condition; //done to lower context switching. Allows dealloc of triggers made in the lambda
    return Trigger(m_drivers, [triggerCpy, trigger]() { return triggerCpy() || trigger(); });
}

Trigger Trigger::operator^(std::function<bool()> trigger)
{
    auto triggerCpy = m_condition; //done to lower context switching. Allows dealloc of triggers made in the lambda
    return Trigger(m_drivers, [triggerCpy, trigger]() { return triggerCpy() ^ trigger(); });
}

Trigger* Trigger::schedule(Command* command, std::vector<Command*>* commands)
{
    commands->emplace_back(command);
    return this;

}  // namespace control

void Trigger::update()
{
    bool value = getAsBoolean();

    bool changed = value != m_lastValue;
    m_lastValue = value;
    m_toggleState = (changed && value) ? !m_toggleState : m_toggleState;

    // no change? no commands get cancelled

    if (!changed) return;
    // schedule commands as needed
    safeSchedule(value ? m_onTrueCommands : m_onFalseCommands);
    safeSchedule(value ? m_whileTrueCommands : m_whileFalseCommands);
   

    // cancel commands as needed
    safeCancel(value ? m_whileFalseCommands : m_whileTrueCommands);

    if(value){
        safeCancel(m_toggleState ? m_toggleOnFalseCommands : m_toggleOnTrueCommands);
        safeSchedule(m_toggleState ? m_toggleOnTrueCommands : m_toggleOnFalseCommands);
    }
}

void Trigger::safeSchedule(std::vector<Command*> commands)
{
    for (auto command : commands)
    {

        m_drivers->commandScheduler.removeCommand(command, true);
        m_drivers->commandScheduler.addCommand(command);
    }
}

void Trigger::safeCancel(std::vector<Command*> commands)
{
    for (auto command : commands)
    {
        m_drivers->commandScheduler.removeCommand(command, true);
    }
}

}  // namespace control
}  // namespace tap