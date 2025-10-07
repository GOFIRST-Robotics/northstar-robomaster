#ifndef SONG_TYPES_HPP
#define SONG_TYPES_HPP

#include <cstdint>
#include <vector>

// --- Core Data Structures ---

/**
 * @brief Represents a single musical note with a duration.
 */
struct songNote
{
    uint8_t noteIndex;     // Index from the NOTE_FREQUENCIES table (0-63)
    uint32_t duration_ms;  // Duration to play the note in milliseconds
};

/**
 * @brief A Song is defined as a sequence (vector) of Notes.
 */
using Song = std::vector<songNote>;

// --- Note Index Constants ---
// Based on the NOTE_FREQUENCIES array where A4 = index 34.

constexpr uint8_t REST = 0;

// Octave 3
constexpr uint8_t NOTE_B3 = 24;

// Octave 4
constexpr uint8_t NOTE_C4 = 25;
constexpr uint8_t NOTE_CS4 = 26;  // C Sharp
constexpr uint8_t NOTE_D4 = 27;
constexpr uint8_t NOTE_DS4 = 28;  // D Sharp
constexpr uint8_t NOTE_E4 = 29;
constexpr uint8_t NOTE_F4 = 30;
constexpr uint8_t NOTE_FS4 = 31;  // F Sharp
constexpr uint8_t NOTE_G4 = 32;
constexpr uint8_t NOTE_GS4 = 33;  // G Sharp
constexpr uint8_t NOTE_A4 = 34;
constexpr uint8_t NOTE_AS4 = 35;  // A Sharp
constexpr uint8_t NOTE_B4 = 36;

// Octave 5
constexpr uint8_t NOTE_C5 = 37;
constexpr uint8_t NOTE_D5 = 39;
constexpr uint8_t NOTE_E5 = 41;

// --- Note Duration Helpers ---
// These functions calculate note durations in milliseconds based on a given tempo.

constexpr uint32_t quarterNote(uint32_t bpm) { return 60000 / bpm; }

constexpr uint32_t eighthNote(uint32_t bpm) { return quarterNote(bpm) / 2; }

constexpr uint32_t halfNote(uint32_t bpm) { return quarterNote(bpm) * 2; }

constexpr uint32_t wholeNote(uint32_t bpm) { return quarterNote(bpm) * 4; }

constexpr uint32_t dottedQuarterNote(uint32_t bpm) { return quarterNote(bpm) + eighthNote(bpm); }

#endif  // SONG_TYPES_HPP