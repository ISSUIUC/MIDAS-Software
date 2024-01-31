#pragma once

#include "buzzer.h"
#include "sensors.h"
#include "rocket_state.h"
#include "finite-state-machines/fsm.h"
#include "data_logging.h"

struct RocketSystems {
    Sensors sensors;
    RocketData rocket_data;
    LogSink log_sink;
    EmmcLog emmc_sink;
    BuzzerController buzzer;
};

void begin_systems(RocketSystems* config);
