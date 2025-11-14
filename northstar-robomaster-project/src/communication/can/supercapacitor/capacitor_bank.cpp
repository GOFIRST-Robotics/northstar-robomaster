/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "capacitor_bank.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace src::can::capbank
{
CapacitorBank::CapacitorBank(
    tap::Drivers* drivers,
    tap::can::CanBus canBus,
    const float capacitance)
    : tap::can::CanRxListener(drivers, CAP_BANK_CAN_ID, canBus),
      capacitance(capacitance),
      powerLimit(0)
{
    currentTXMessageState = {
        .enable_module = false,
        .reset = false,
        .pow_limit = 0,
        .energy_buffer = 0};
    lastCapData = {.chassis_power = 0.0f, .error = 0, .cap_energy = 0};
}

void CapacitorBank::processMessage(const modm::can::Message& message)
{
    std::memcpy(&lastCapData.chassis_power, &message.data[0], sizeof(float));
    lastCapData.error = message.data[4];
    lastCapData.cap_energy = message.data[5];
}

void CapacitorBank::sendTXMessage(TXcapMessage msg)
{
    const size_t data_length = sizeof(TXcapMessage);
    modm::can::Message message(CAP_BANK_CAN_ID, data_length);
    std::memcpy(message.data, &msg, data_length);
    this->drivers->can.sendMessage(this->canBus, message);
}

void CapacitorBank::initialize()
{
    this->attachSelfToRxHandler();
    this->heartbeat.restart(0);
}

void CapacitorBank::start()
{
    currentTXMessageState.enable_module = true;
    sendTXMessage(currentTXMessageState);
}

void CapacitorBank::stop()
{
    currentTXMessageState.enable_module = false;
    sendTXMessage(currentTXMessageState);
}

void CapacitorBank::setPowerLimit(uint8_t watts)
{
    currentTXMessageState.pow_limit = watts;
    sendTXMessage(currentTXMessageState);
}

void CapacitorBank::setEnergyBuffer(uint16_t energyBuffer)
{
    currentTXMessageState.energy_buffer = energyBuffer;
    sendTXMessage(currentTXMessageState);
}

bool CapacitorBank::isEnabled() const { return currentTXMessageState.enable_module; }

bool CapacitorBank::canSprint() const
{
    return getAvailableEnergy() >= CAPACITOR_SPRINT_THRESHOLD_PERCENT;
}

}  // namespace src::can::capbank
