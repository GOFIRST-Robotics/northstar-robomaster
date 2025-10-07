#ifndef BUZZER_SUBSYSTEM_HPP
#define BUZZER_SUBSYSTEM_HPP

#include "tap/communication/gpio/pwm.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
namespace src::control::buzzer
{
class BuzzerSubsystem : public tap::control::Subsystem
{
public:
    /**
     * @brief Constructs the BuzzerSubsystem.
     *
     * @param drivers A pointer to the global drivers object.
     */
    explicit BuzzerSubsystem(tap::Drivers* drivers);

    /**
     * @brief Plays a single note immediately.
     * * The note will play indefinitely until stop() or another note is played.
     *
     * @param noteIndex The index of the note to play (0-63).
     */
    void playNote(uint8_t noteIndex);

    /**
     * @brief Stops any sound currently playing on the buzzer.
     */
    void stop();

private:
    // Note value 34 is assigned to A4 = 440Hz and note value 0 means silence, making the
    // range 1 = C2 -> 63 = D7
    // generated in python with `440 * 2**((np.arange(64)-34)/12)`
    static constexpr float NOTE_FREQUENCIES[] = {
        0.0,           65.40639133,   69.29565774,   73.41619198,   77.78174593,   82.40688923,
        87.30705786,   92.49860568,   97.998859,     103.82617439,  110.,          116.54094038,
        123.47082531,  130.81278265,  138.59131549,  146.83238396,  155.56349186,  164.81377846,
        174.61411572,  184.99721136,  195.99771799,  207.65234879,  220.,          233.08188076,
        246.94165063,  261.6255653,   277.18263098,  293.66476792,  311.12698372,  329.62755691,
        349.22823143,  369.99442271,  391.99543598,  415.30469758,  440.,          466.16376152,
        493.88330126,  523.2511306,   554.36526195,  587.32953583,  622.25396744,  659.25511383,
        698.45646287,  739.98884542,  783.99087196,  830.60939516,  880.,          932.32752304,
        987.76660251,  1046.5022612,  1108.73052391, 1174.65907167, 1244.50793489, 1318.51022765,
        1396.91292573, 1479.97769085, 1567.98174393, 1661.21879032, 1760.,         1864.65504607,
        1975.53320502, 2093.0045224,  2217.46104781, 2349.31814334};
};

// Calculate the number of notes in the array for bounds checking
static constexpr size_t NUM_NOTES = 64;

}  // namespace src::control::buzzer

#endif  // BUZZER_SUBSYSTEM_HPP