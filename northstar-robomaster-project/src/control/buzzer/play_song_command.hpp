#ifndef PLAY_SONG_COMMAND_HPP
#define PLAY_SONG_COMMAND_HPP

#include "tap/control/command.hpp"

#include "buzzer_subsystem.hpp"
#include "song_types.hpp"

namespace src::control::buzzer
{
class PlaySongCommand : public tap::control::Command
{
public:
    PlaySongCommand(BuzzerSubsystem* buzzer, const Song& song);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "PlaySongCommand"; }

private:
    BuzzerSubsystem* buzzer;
    const Song songToPlay;

    size_t currentNoteIndex;
    uint32_t noteTimeRemaining_ms;
};
}  // namespace src::control::buzzer

#endif  // PLAY_SONG_COMMAND_HPP