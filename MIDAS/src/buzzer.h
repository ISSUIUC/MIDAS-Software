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
    static constexpr uint32_t MAX_TUNE_LENGTH = 32;
    Sound tune_buffer_[MAX_TUNE_LENGTH];
    uint32_t index_ = 0;
    uint32_t length_ = 0;

    bool new_tune_started = false;
    uint32_t when_sound_started_ = 0;

    void tick_sounds();

public:
    BuzzerController() = default;

    ErrorCode init();
    void tick();
    void report_beeps(bool* cont, bool fsm_fail); // Beeps that report system status
    void play_tune(const Sound* tune, uint32_t length);
    bool is_playing();
};

#define FREE_BIRD_LENGTH 48
#define JAMES_BOND_LENGTH 27
#define WARN_TONE_LENGTH 1
#define LAND_TONE_LENGTH 11

#define C_XL_LENGTH 3
#define C_MG_LENGTH 5

extern Sound free_bird[FREE_BIRD_LENGTH];
extern Sound james_bond[JAMES_BOND_LENGTH];
extern Sound warn_tone[WARN_TONE_LENGTH];
extern Sound land_tone[LAND_TONE_LENGTH];

extern Sound xl_calib_rdy[C_XL_LENGTH];
extern Sound xl_calib_next_axis[C_XL_LENGTH];
extern Sound xl_calib_done[C_XL_LENGTH];
extern Sound xl_calib_abort[C_XL_LENGTH];

extern Sound mg_calib_rdy[C_MG_LENGTH];
extern Sound mg_calib_done[C_MG_LENGTH];
extern Sound mg_calib_inp[C_MG_LENGTH];
extern Sound mg_calib_bad[C_MG_LENGTH];