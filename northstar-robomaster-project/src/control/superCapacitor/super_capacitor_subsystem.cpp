#include "super_capacitor_subsystem.hpp"

#include "capacitor_constants.hpp"

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

}  // namespace src::capacitor