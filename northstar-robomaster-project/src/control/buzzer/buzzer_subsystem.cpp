#include "buzzer_subsystem.hpp"

#include <cstddef>  // Required for size_t

#include "song_types.hpp"


namespace src::control::buzzer
{
BuzzerSubsystem::BuzzerSubsystem(tap::Drivers* drivers) : Subsystem(drivers) {}

void BuzzerSubsystem::playNote(uint8_t noteIndex)
{
    if (noteIndex >= NUM_NOTES)
    {
        // Play silence if the note index is invalid
        tap::buzzer::playNote(&(drivers->pwm), 0);
        return;
    }
    const float frequency = NOTE_FREQUENCIES[noteIndex];
    tap::buzzer::playNote(&(drivers->pwm), static_cast<uint32_t>(frequency));
}

void BuzzerSubsystem::stop() { tap::buzzer::playNote(&(drivers->pwm), 0); }
}  // namespace src::control::buzzer