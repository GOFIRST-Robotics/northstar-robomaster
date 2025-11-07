#ifndef TSN_SONG_HPP
#define TSN_SONG_HPP

#include "control/buzzer/song_types.hpp"

const Song tsnSong = {
    {REST, 150},
    {NOTE_G4, 80},
    {NOTE_C5, 80},
    {NOTE_E5, 80},
    {NOTE_G5, 400},
};

constexpr uint32_t TWR_BPM = 190;
constexpr uint32_t QUARTER = quarterNote(TWR_BPM);
constexpr uint32_t EIGHTH = QUARTER / 2;
constexpr uint32_t QUARTER = EIGHTH / 2;
constexpr uint32_t WHOLE = QUARTER * 4;

const Song theWorldRevolving{
    {QUARTER, REST},
    {WHOLE, NOTE_C6},
    {EIGHTH, REST},
    {EIGHTH, NOTE_C6},
    {EIGHTH, NOTE_B5},
    {EIGHTH, NOTE_C6},
    {QUARTER, NOTE_G6},
    {EIGHTH, NOTE_B5},
    {WHOLE + QUARTER, NOTE_C6},
    {EIGHTH, NOTE_C6},
    {EIGHTH, NOTE_B5},
    {EIGHTH, NOTE_A5},
    {QUARTER, NOTE_G5},
    {QUARTER, NOTE_B5},
    {EIGHTH, NOTE_C6},
    {EIGHTH, NOTE_B5},
    {EIGHTH, NOTE_C6},
    {QUARTER + EIGHTH, NOTE_G6},
    {EIGHTH, NOTE_G6},
    {EIGHTH, NOTE_A6},
    {QUARTER, NOTE_G6},
    {EIGHTH, NOTE_F6},
    {QUARTER + EIGHTH, NOTE_E6},
    {QUARTER, NOTE_C6},
    {QUARTER + EIGHTH, NOTE_D6},
    {QUARTER + EIGHTH, NOTE_A5},
    {QUARTER, NOTE_D6},
    {QUARTER + EIGHTH, NOTE_E6},
    {SIXTEENTH, NOTE_F6},
    {SIXTEENTH, NOTE_E6},
    {QUARTER + QUARTER, NOTE_D6},
};

#endif