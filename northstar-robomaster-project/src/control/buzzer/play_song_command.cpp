#include "play_song_command.hpp"

namespace src::control::buzzer
{
PlaySongCommand::PlaySongCommand(BuzzerSubsystem* buzzer, const Song& song)
    : buzzer(buzzer),
      songToPlay(song),
      currentNoteIndex(0),
      noteTimeRemaining_ms(0)
{
    addSubsystemRequirement(buzzer);
}

void PlaySongCommand::initialize()
{
    if (songToPlay.empty())
    {
        return;
    }
    // Reset state and start the first note.
    currentNoteIndex = 0;
    const songNote& firstNote = songToPlay[currentNoteIndex];
    noteTimeRemaining_ms = firstNote.duration_ms;
    buzzer->playNote(firstNote.noteIndex);
}

// Called repeatedly every 2ms by the scheduler.
void PlaySongCommand::execute()
{
    if (isFinished())
    {
        return;
    }

    if (noteTimeRemaining_ms > tap::Drivers::DT)
    {
        noteTimeRemaining_ms -= tap::Drivers::DT;
    }
    else
    {
        // Time for the current note is up, advance to the next note.
        currentNoteIndex++;

        if (!isFinished())
        {
            // Load and play the next note.
            const songNote& nextNote = songToPlay[currentNoteIndex];
            noteTimeRemaining_ms = nextNote.duration_ms;
            buzzer->playNote(nextNote.noteIndex);
        }
    }
}

// Returns true when the command should stop.
bool PlaySongCommand::isFinished() const
{
    return songToPlay.empty() || currentNoteIndex >= songToPlay.size();
}

// Called once when the command ends (either by finishing or being interrupted).
void PlaySongCommand::end(bool interrupted)
{
    // Always stop the buzzer when the command is over to ensure silence.
    buzzer->stop();
}
}  // namespace src::control::buzzer