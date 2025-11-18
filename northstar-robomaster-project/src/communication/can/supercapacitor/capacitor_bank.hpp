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

#ifndef CAPACITOR_BANK_HPP_
#define CAPACITOR_BANK_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/control/chassis/power_limiter.hpp"
#include "tap/drivers.hpp"

#include "modm/architecture/interface/can_message.hpp"
#include "modm/math/interpolation/linear.hpp"
#include "robot/control_operator_interface.hpp"

#include "capacitor_constants.hpp"

namespace src::can::capbank
{
struct TXcapMessage
{
    uint8_t enable_module;
    uint8_t reset;
    uint8_t pow_limit;
    uint16_t energy_buffer;
};

struct RXcapMessage
{
    float chassis_power;
    uint8_t error;
    uint8_t cap_energy;
};

enum State
{
    UNKNOWN = -1,
    RESET = 0,
    SAFE = 1,
    CHARGE = 2,
    CHARGE_DISCHARGE = 3,
    DISCHARGE = 4,
    BATTERY_OFF = 5,
    DISABLED = 6,
};

enum SprintMode
{
    NO_SPRINT = 0,
    HALF_SPRINT = 1,
    SPRINT = 2
};
class CapacitorBank : public tap::can::CanRxListener
{
public:
    CapacitorBank(
        tap::Drivers* drivers,
        tap::can::CanBus canBus,
        const float capacitance,
        src::control::ControlOperatorInterface* operatorInterface);

    void processMessage(const modm::can::Message& message) override;

    mockable void initialize();

    void sendTXMessage(TXcapMessage msg);

    void start();
    void stop();
    void update();
    void setPowerLimit(uint8_t watts);
    void setEnergyBuffer(uint16_t energyBuffer);

public:
    int getAvailableEnergy() const { return this->lastCapData.cap_energy; };
    int getPowerLimit() const { return this->powerLimit; };
    uint16_t getAllowedSprintWattage() const { return ALLOWED_SPRINT_WATTAGE; }

    bool isEnabled() const;

    bool canSprint() const;

    bool isSprinting();

#ifndef ENV_UNIT_TESTS
private:
#endif
    const float capacitance;

    TXcapMessage currentTXMessageState;

    RXcapMessage lastCapData;

    uint8_t powerLimit = 0;

    tap::arch::MilliTimeout heartbeat;

    src::control::ControlOperatorInterface* operatorInterface;
};
}  // namespace src::can::capbank

#endif  // CAPACITOR_BANK_HPP_
