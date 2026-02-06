#ifndef CAPACITOR_CONSTANTS_HPP_
#define CAPACITOR_CONSTANTS_HPP_

#include <stdint.h>

namespace src::capacitor
{
constexpr uint16_t CAP_BANK_CAN_ID = 0x1EC;
const uint8_t CAPACITOR_SPRINT_THRESHOLD_PERCENT = 25.5;  // from 0-255
const uint16_t ALLOWED_SPRINT_WATTAGE = 100;
const float SUPERCAPACITOR_CAPACITANCE = 1.0;
}  // namespace src::capacitor

#endif  // CAPACITOR_CONSTANTS_HPP_