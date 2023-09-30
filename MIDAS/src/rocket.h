#pragma once

#include "sensors.h"
#include "rocket_state.h"


struct RocketConfig {
    Sensors sensors{};
    RocketState rocket_state{};
};

void start_rocket(RocketConfig& config);
