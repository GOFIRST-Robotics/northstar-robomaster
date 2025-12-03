// #ifndef MEGALOVANIA_SONG_HPP
// #define MEGALOVANIA_SONG_HPP

// #include "control/buzzer/song_types.hpp"

// // "Megalovania" has a tempo of about 120 BPM for the main riff.
// constexpr uint32_t MEGALOVANIA_BPM = 120;

// // Helper durations for this specific song to make it more readable
// constexpr uint32_t SIXTEENTH = eighthNote(MEGALOVANIA_BPM) / 2;
// constexpr uint32_t DOTTED_EIGHTH = SIXTEENTH * 3;
// constexpr uint32_t EIGHTH = eighthNote(MEGALOVANIA_BPM);

// const Song megalovaniaSong = {
//     {REST, SIXTEENTH},
//     // Measure 1
//     {NOTE_D4, SIXTEENTH},
//     {NOTE_D4, SIXTEENTH},
//     {NOTE_D5, EIGHTH},
//     {NOTE_A4, EIGHTH},
//     {REST, SIXTEENTH},
//     {NOTE_GS4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_F4, EIGHTH},
//     {NOTE_D4, SIXTEENTH},
//     {NOTE_F4, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},

//     // Measure 2
//     {NOTE_C4, SIXTEENTH},
//     {NOTE_C4, SIXTEENTH},
//     {NOTE_D5, EIGHTH},
//     {NOTE_A4, EIGHTH},
//     {REST, SIXTEENTH},
//     {NOTE_GS4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_F4, EIGHTH},
//     {NOTE_D4, SIXTEENTH},
//     {NOTE_F4, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},

//     // Measure 3
//     {NOTE_B3, SIXTEENTH},
//     {NOTE_B3, SIXTEENTH},
//     {NOTE_D5, EIGHTH},
//     {NOTE_A4, EIGHTH},
//     {REST, SIXTEENTH},
//     {NOTE_GS4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_F4, EIGHTH},
//     {NOTE_D4, SIXTEENTH},
//     {NOTE_F4, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},

//     // Measure 4
//     {NOTE_CS3, SIXTEENTH},
//     {NOTE_CS3, SIXTEENTH},
//     {NOTE_D5, EIGHTH},
//     {NOTE_A4, EIGHTH},
//     {REST, SIXTEENTH},
//     {NOTE_GS4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_F4, EIGHTH},
//     {NOTE_D4, SIXTEENTH},
//     {NOTE_F4, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},

//     // Measure 5 Repeat
//     {NOTE_D4, SIXTEENTH},
//     {NOTE_D4, SIXTEENTH},
//     {NOTE_D5, EIGHTH},
//     {NOTE_A4, EIGHTH},
//     {REST, SIXTEENTH},
//     {NOTE_GS4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_F4, EIGHTH},
//     {NOTE_D4, SIXTEENTH},
//     {NOTE_F4, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},

//     // Measure 6
//     {NOTE_C4, SIXTEENTH},
//     {NOTE_C4, SIXTEENTH},
//     {NOTE_D5, EIGHTH},
//     {NOTE_A4, EIGHTH},
//     {REST, SIXTEENTH},
//     {NOTE_GS4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_F4, EIGHTH},
//     {NOTE_D4, SIXTEENTH},
//     {NOTE_F4, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},

//     // Measure 7
//     {NOTE_B3, SIXTEENTH},
//     {NOTE_B3, SIXTEENTH},
//     {NOTE_D5, EIGHTH},
//     {NOTE_A4, EIGHTH},
//     {REST, SIXTEENTH},
//     {NOTE_GS4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_F4, EIGHTH},
//     {NOTE_D4, SIXTEENTH},
//     {NOTE_F4, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},

//     // Measure 8
//     {NOTE_CS3, SIXTEENTH},
//     {NOTE_CS3, SIXTEENTH},
//     {NOTE_D5, EIGHTH},
//     {NOTE_A4, EIGHTH},
//     {REST, SIXTEENTH},
//     {NOTE_GS4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},
//     {REST, SIXTEENTH},
//     {NOTE_F4, EIGHTH},
//     {NOTE_D4, SIXTEENTH},
//     {NOTE_F4, SIXTEENTH},
//     {NOTE_G4, SIXTEENTH},
// };

// #endif  // MEGALOVANIA_SONG_HPP