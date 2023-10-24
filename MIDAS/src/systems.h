#pragma once

#include "sensors.h"
#include "rocket_state.h"
#include "finite-state-machines/fsm.h"


struct RocketSystems {
    Sensors sensors{};
    RocketState rocket_state{};
};

void begin_systems(RocketSystems& config);
