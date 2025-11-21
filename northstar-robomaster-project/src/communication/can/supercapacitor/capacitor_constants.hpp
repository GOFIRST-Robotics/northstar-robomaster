#include <stdint.h>

#define MAP_POWER(x) (limitval<uint_8>(x, -100, 100) * 2.55f)

namespace src::can::capbank
{
constexpr uint16_t CAP_BANK_CAN_ID = 0x1EC;
const uint8_t CAPACITOR_SPRINT_THRESHOLD_PERCENT = 25;  // Mapped from 0-255
const uint16_t ALLOWED_SPRINT_WATTAGE = 100;
}  // namespace src::can::capbank