#pragma once
#include "hardware_interface.h"

class BuzzerBackend final : public IBuzzerBackend {
    void init() override;
    void play_tone(uint32_t frequency) override;
};
