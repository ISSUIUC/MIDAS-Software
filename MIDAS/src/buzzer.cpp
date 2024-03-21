#include "buzzer.h"
#include "notes.h"

#define BUZZER_PIN (48)
#define BUZZER_CHANNEL (1)

void BuzzerController::play_tune(Sound* tune, uint32_t length) {
    current_tune_ = tune;
    index_ = 0;
    length_ = length;
    new_tune_started = true;
}

void BuzzerController::tick() {
    tick_sounds();
}

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

ErrorCode BuzzerController::init() {
    // ledcDetachPin(BUZZER_PIN);  // this probably isn't necessary but who am I do question the knowledge of github
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
    return NoError;
}

#define MS_PER_4BEAT 4000


Sound free_bird[FREE_BIRD_LENGTH] = {/*measure 1*/ d4_eight, g4_eight, d4_eight,
    /*measure 2*/ f_nat_4_eight, g4_eight, f_nat_4_quart, rest, f_nat_4_quart, rest, f_nat_4_eight, d4_eight
};