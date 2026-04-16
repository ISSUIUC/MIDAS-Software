#include "buzzer.h"
#include <hardware/pins.h>

/**
 * @brief starts playing a new song
 * 
 * @param tune Song to be played
 * @param length Length of song to be played
*/
void BuzzerController::play_tune(Sound* tune, uint32_t length) {
    current_tune_ = tune;
    index_ = 0;
    length_ = length;
    new_tune_started = true;
}

/**
 * @brief public interface to tick the buzzer
*/
void BuzzerController::tick() {
    tick_sounds();
}

/**
 * @brief Returns whether the buzzer is currently playing a tune
 */
bool BuzzerController::is_playing() {
    return (current_tune_ != nullptr);
}

/**
 * @brief ticks the bizzer, plays next note/ starts new song if applicable
*/
void BuzzerController::tick_sounds() {
    if (current_tune_ == nullptr) {
        return;
    }

    Sound& current_sound = current_tune_[index_];

    uint32_t current_time = pdTICKS_TO_MS(xTaskGetTickCount());

    if (new_tune_started) {
        Sound& next_sound = current_tune_[index_];
        ledcWriteTone(BUZZER_CHANNEL, next_sound.frequency);

        when_sound_started_ = current_time;
        new_tune_started = false;
    } else if (current_time - when_sound_started_ >= current_sound.duration_ms) {
        index_++;
        if (index_ >= length_) {
            current_tune_ = nullptr;
            index_ = 0;
            length_ = 0;

            ledcWriteTone(BUZZER_CHANNEL, 0);
        } else {
            Sound& next_sound = current_tune_[index_];
            ledcWriteTone(BUZZER_CHANNEL, next_sound.frequency);

            when_sound_started_ = current_time;
        }
    }
}

/**
 * @brief Initializes buzzer
 * 
 * @return Error code
*/
ErrorCode BuzzerController::init() {
    // ledcDetachPin(BUZZER_PIN);  // this probably isn't necessary but who am I do question the knowledge of github

    // This is done for a startup tone before begin_systems, but it doesn't hurt to do it again ig.
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
    return NoError;
}

/**
 * @brief notes to use for creating a song, along with a tempo
*/
#define MS_PER_4BEAT 6000

#define b_flat_4_eight Sound{466, static_cast<uint8_t>(0.125 * MS_PER_4BEAT)}

#define e4_eight Sound{330, static_cast<uint8_t>(0.125 * MS_PER_4BEAT)}
#define d4_quart Sound{294, static_cast<uint8_t>(0.25 * MS_PER_4BEAT)}
#define g4_quart Sound{392, static_cast<uint8_t>(0.25 * MS_PER_4BEAT)}
#define f_nat_4_quart Sound{350, static_cast<uint8_t>(0.25 * MS_PER_4BEAT)}

#define b_flat_4_quart Sound{466, static_cast<uint8_t>(0.25 * MS_PER_4BEAT)}
#define e4_quart Sound{330, static_cast<uint8_t>(0.25 * MS_PER_4BEAT)}
#define d4_fifth Sound{294, static_cast<uint8_t>(0.05 * MS_PER_4BEAT)}
#define f_nat_4_fifth Sound{350, static_cast<uint8_t>(0.05 * MS_PER_4BEAT)}
#define d4_2fifth Sound{294, static_cast<uint8_t>(0.1 * MS_PER_4BEAT)}
#define f_nat_4_2fifth Sound{350, static_cast<uint8_t>(0.1 * MS_PER_4BEAT)}

#define d4_eight Sound{294, static_cast<uint8_t>(240/1.3)}
#define g4_eight Sound{392, static_cast<uint8_t>(240/1.3)}
#define something Sound{393, static_cast<uint8_t>(255/1.3)}
#define somethingdiv2 Sound{393, static_cast<uint8_t>(220/1.3)}
#define f_nat_4_quart2 Sound{350, static_cast<uint8_t>(220/1.3)}
#define f_nat_4_eight2 Sound{350, static_cast<uint8_t>(220/1.3)}
#define d4_eight2 Sound{294, static_cast<uint8_t>(220/1.3)}
#define g4_eight2 Sound{392, static_cast<uint8_t>(220/1.3)}
#define high Sound{467, static_cast<uint8_t>(150/1.3)}
#define low Sound{393, static_cast<uint8_t>(200/1.3)}

#define longrest Sound{0, 0}
#define rest Sound{0, static_cast<uint8_t>(10/1)}

#define WARN_TONE_PITCH Sound{2730, 125} // 2730 is the resonant frequency of the buzzer, thus it will be loudest here.
#define WARN_TONE_BLANK Sound{0, 125}

#define LAND_TONE_PITCH Sound{2730, 250}
#define LAND_TONE_WAIT Sound{0, 250}

/**
 * @brief free bird solo song, to be played on startup/ second stage iginition
*/

Sound free_bird[FREE_BIRD_LENGTH] = {d4_eight2, rest, g4_eight2, rest, d4_eight2, 
    something, something, rest, something, something, rest, something, something, 
    f_nat_4_quart2, rest, d4_eight2, rest, f_nat_4_eight2, rest, somethingdiv2, rest, f_nat_4_eight2, rest, d4_eight2, rest, somethingdiv2, somethingdiv2, 
};

// rest of the song
// rest, f_nat_4_quart2, rest, d4_eight2, somethingdiv2, somethingdiv2, rest, rest, rest, high, high, low, low, rest, high, high, low, low, rest, high, high, low, low

/**
 * @brief James Bond theme, only to be played on Midas Mini 007
 */
Sound james_bond[JAMES_BOND_LENGTH] = {Sound{330, static_cast<uint8_t>(240)}, Sound{392, static_cast<uint8_t>(255)}, Sound{392, static_cast<uint8_t>(109)}, Sound{622, static_cast<uint8_t>(148)}, Sound{587, static_cast<uint8_t>(255)}, Sound{587, static_cast<uint8_t>(255)}, Sound{587, static_cast<uint8_t>(112)}, Sound{392, static_cast<uint8_t>(174)}, Sound{466, static_cast<uint8_t>(255)}, Sound{466, static_cast<uint8_t>(3)}, Sound{494, static_cast<uint8_t>(255)}, Sound{494, static_cast<uint8_t>(255)}, Sound{494, static_cast<uint8_t>(255)}, Sound{494, static_cast<uint8_t>(255)}, Sound{494, static_cast<uint8_t>(140)}, Sound{392, static_cast<uint8_t>(255)}, Sound{392, static_cast<uint8_t>(33)}, Sound{440, static_cast<uint8_t>(64)}, Sound{392, static_cast<uint8_t>(64)}, Sound{370, static_cast<uint8_t>(255)}, Sound{370, static_cast<uint8_t>(255)}, Sound{370, static_cast<uint8_t>(255)}, Sound{370, static_cast<uint8_t>(67)}, Sound{330, static_cast<uint8_t>(255)}, Sound{330, static_cast<uint8_t>(1)}, Sound{277, static_cast<uint8_t>(255)}, Sound{277, static_cast<uint8_t>(245)}};



/**
 * @brief Warn tone, to be played in "unsafe" non-flight states (STATE_IDLE, STATE_PYRO_TEST)
 */
Sound warn_tone[WARN_TONE_LENGTH] = {WARN_TONE_PITCH};

/**
 * @brief Land state tone, played whenever the board is in the "LANDED" state to provide a audio indicator to recovery parties
 */
Sound land_tone[LAND_TONE_LENGTH] = {WARN_TONE_PITCH, WARN_TONE_PITCH, LAND_TONE_WAIT, LAND_TONE_WAIT, LAND_TONE_WAIT, LAND_TONE_WAIT, LAND_TONE_WAIT, LAND_TONE_WAIT, LAND_TONE_WAIT, LAND_TONE_WAIT};