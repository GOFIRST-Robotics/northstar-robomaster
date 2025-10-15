#ifndef TSN_SONG_HPP
#define TSN_SONG_HPP

#include "control/buzzer/song_types.hpp"

const Song tsnSong = {
    {NOTE_C3, 150}, {NOTE_E3, 150}, {NOTE_G3, 150}, {NOTE_B3, 200}, {NOTE_C4, 300}, {REST, 100},
    {NOTE_C4, 120}, {NOTE_E4, 120}, {NOTE_G4, 120}, {NOTE_B4, 150}, {NOTE_C5, 300}, {REST, 80},
    {NOTE_F4, 100}, {NOTE_G4, 100}, {NOTE_A4, 100}, {NOTE_C5, 250}, {NOTE_E5, 400}, {REST, 150},
    {NOTE_G4, 80},  {NOTE_C5, 80},  {NOTE_E5, 80},  {NOTE_G5, 400},
};

#endif