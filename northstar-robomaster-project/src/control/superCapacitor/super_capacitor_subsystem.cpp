#include "super_capacitor_subsystem.hpp"

namespace src::capacitor
{
SuperCapacitor::SuperCapacitor()
{
    capacitorBank = CapacitorBank(
        drivers(),
        CanBus::CAN_BUS2,
        SUPER_CAPACITOR_CAPACITANCE,
        &drivers()->controlOperatorInterface);
}
}  // namespace src::capacitor