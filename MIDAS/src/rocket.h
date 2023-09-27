#pragma once

#include "sensors.h"
#include "rocket_state.h"


struct RocketConfig {
public:
    Sensors sensors;
    RocketState rocket_state;

    RocketConfig() = delete;
};

void start_threads(RocketConfig config);
