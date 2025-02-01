#pragma once
#include "hardware_interface.h"

struct PyroBackend final: IPyroBackend {
    ErrorCode init() override;
    void arm_all() override;
    void fire_channel(int channel) override;
};
