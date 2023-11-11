#include "buzzer.h"

#define BUZZER_PIN (0xDE)

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
        ledcWriteTone(BUZZER_PIN, next_sound.frequency);

        when_sound_started_ = current_time;
        new_tune_started = false;
    } else if (current_time - when_sound_started_ >= current_sound.duration_ms) {
        index_++;
        if (index_ >= length_) {
            current_tune_ = nullptr;
            index_ = 0;
            length_ = 0;
        } else {
            Sound& next_sound = current_tune_[index_];
            ledcWriteTone(BUZZER_PIN, next_sound.frequency);

            when_sound_started_ = current_time;
        }
    }
}
