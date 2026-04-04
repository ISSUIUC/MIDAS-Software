#include "buzzer.h"
#include <cstring>
#include <hardware/pins.h>

/**
 * @brief starts playing a new song
 * 
 * @param tune Song to be played
 * @param length Length of song to be played
*/
void BuzzerController::play_tune(const Sound* tune, uint32_t length) {
    if (length > MAX_TUNE_LENGTH) length = MAX_TUNE_LENGTH;
    memcpy(tune_buffer_, tune, length * sizeof(Sound));
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
    return (length_ > 0);
}

/**
 * @brief ticks the bizzer, plays next note/ starts new song if applicable
*/
void BuzzerController::tick_sounds() {
    if (length_ == 0) {
        return;
    }

    Sound& current_sound = tune_buffer_[index_];

    uint32_t current_time = pdTICKS_TO_MS(xTaskGetTickCount());

    if (new_tune_started) {
        Sound& next_sound = tune_buffer_[index_];
        ledcWriteTone(BUZZER_CHANNEL, next_sound.frequency);

        when_sound_started_ = current_time;
        new_tune_started = false;
    } else if (current_time - when_sound_started_ >= current_sound.duration_ms) {
        index_++;
        if (index_ >= length_) {
            index_ = 0;
            length_ = 0;

            ledcWriteTone(BUZZER_CHANNEL, 0);
        } else {
            Sound& next_sound = tune_buffer_[index_];
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

#define rest Sound{0, 10}
#define d4_eight Sound{294, static_cast<uint8_t>(0.125 * MS_PER_4BEAT)}
#define g4_eight Sound{392, static_cast<uint8_t>(0.125 * MS_PER_4BEAT)}
#define f_nat_4_eight Sound{350, static_cast<uint8_t>(0.125 * MS_PER_4BEAT)}
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

#define WARN_TONE_PITCH Sound{2730, 125} // 2730 is the resonant frequency of the buzzer, thus it will be loudest here.
#define WARN_TONE_BLANK Sound{0, 125}

#define LAND_TONE_PITCH Sound{2730, 250}
#define LAND_TONE_WAIT Sound{0, 250}

#define REPORT_WARN Sound{2730, 200}
#define REPORT_CONTHIGH Sound{3000, 100}
#define REPORT_CONTLOW Sound{2500, 100}
#define REPORT_PAUSES Sound{0, 50}
#define REPORT_PAUSEL Sound{0, 200}

#define REPORT_ERRFSM0 Sound{3000, 150}
#define REPORT_ERRFSM1 Sound{2800, 150}

/**
 * @brief free bird solo song, to be played on startup/ second stage iginition
*/
Sound free_bird[FREE_BIRD_LENGTH] = {/*measure 1*/ d4_eight, g4_eight, d4_eight,
    /*measure 2*/ f_nat_4_eight, g4_eight, f_nat_4_quart, rest, f_nat_4_quart, rest, f_nat_4_eight, d4_eight
};

/**
 * @brief Warn tone, to be played in "unsafe" non-flight states (STATE_IDLE, STATE_PYRO_TEST)
 */
Sound warn_tone[WARN_TONE_LENGTH] = {WARN_TONE_PITCH};

/**
 * @brief Land state tone, played whenever the board is in the "LANDED" state to provide a audio indicator to recovery parties
 */
Sound land_tone[LAND_TONE_LENGTH] = {WARN_TONE_PITCH, WARN_TONE_PITCH, LAND_TONE_WAIT, LAND_TONE_WAIT, LAND_TONE_WAIT, LAND_TONE_WAIT, LAND_TONE_WAIT, LAND_TONE_WAIT, LAND_TONE_WAIT, LAND_TONE_WAIT};

void BuzzerController::report_beeps(bool* cont, bool fsm_fail) {
    constexpr uint8_t report_tone_len = 16;
    // Build the tone based off of report data
    
    Sound report_tone[report_tone_len] = {
        REPORT_WARN,
        REPORT_PAUSEL,
        cont[0] ? REPORT_CONTHIGH : REPORT_CONTLOW, REPORT_PAUSES,
        cont[1] ? REPORT_CONTHIGH : REPORT_CONTLOW, REPORT_PAUSES,
        cont[2] ? REPORT_CONTHIGH : REPORT_CONTLOW, REPORT_PAUSES,
        cont[3] ? REPORT_CONTHIGH : REPORT_CONTLOW, REPORT_PAUSES,
        REPORT_PAUSEL,
        fsm_fail ? REPORT_ERRFSM0 : REPORT_PAUSES,
        fsm_fail ? REPORT_ERRFSM1 : REPORT_PAUSES,
        fsm_fail ? REPORT_ERRFSM0 : REPORT_PAUSES,
        fsm_fail ? REPORT_ERRFSM1 : REPORT_PAUSES,
        REPORT_PAUSES
    };

    play_tune(report_tone, report_tone_len);
}