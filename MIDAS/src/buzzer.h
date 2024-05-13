#pragma once

#include <cstdint>
#include "hal.h"
#include "errors.h"

/**
 * @struct Sound
 * 
 * @brief contains information for a single note to play on the buzzer
*/
struct Sound {
public:
    uint32_t frequency;
    uint8_t duration_ms;
};

/**
 * @struct BuzzerController
 * 
 * @brief wraps the buzzer functionality
*/
struct BuzzerController {
private:
    Sound* current_tune_ = nullptr;
    uint32_t index_ = 0;
    uint32_t length_ = 0;

    bool new_tune_started = false;
    uint32_t when_sound_started_ = 0;

    void tick_sounds();

public:
    BuzzerController() = default;

    ErrorCode init();
    void tick();
    void play_tune(Sound* tune, uint32_t length);
};

#define FREE_BIRD_LENGTH 11
extern Sound free_bird[FREE_BIRD_LENGTH];
