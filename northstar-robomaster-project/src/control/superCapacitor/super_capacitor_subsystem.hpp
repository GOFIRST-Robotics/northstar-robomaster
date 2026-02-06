#ifndef SUPER_CAPACITOR_SUBSYSTEM_HPP_
#define SUPER_CAPACITOR_SUBSYSTEM_HPP_

#include "communication/can/supercapacitor/capacitor_bank.hpp"

namespace src::capacitor
{
class SuperCapacitor
{
public:
    SuperCapacitor(tap::Drivers* drivers, tap::can::CanBus canBus);
    void sprint();
    void stopSprinting();
    bool getIsSprinting();
    float getAllowedSprintWattage() { return ALLOWED_SPRINT_WATTAGE; }

private:
    src::can::capbank::CapacitorBank capacitorbank;
    bool isSprinting;
};
}  // namespace src::capacitor

#endif