#include "super_capacitor_subsystem.hpp"

namespace src::capacitor
{
SuperCapacitor::SuperCapacitor(tap::Drivers* drivers, tap::can::CanBus canBus)
    : capacitorbank(
          drivers,
          canBus,
          capacitor::CAP_BANK_CAN_ID,
          capacitor::SUPERCAPACITOR_CAPACITANCE),
      isSprinting(false)
{
}

void SuperCapacitor::sprint() { isSprinting = true; }

void SuperCapacitor::stopSprinting() { isSprinting = false; }

bool SuperCapacitor::getIsSprinting()
{
    return isSprinting && capacitorbank.canSprint(CAPACITOR_SPRINT_THRESHOLD_PERCENT);
}

}  // namespace src::capacitor