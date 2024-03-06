#pragma once

#include "buzzer.h"
#include "sensors.h"
#include "rocket_state.h"
#include "finite-state-machines/fsm.h"
#include "data_logging.h"
#include "telemetry.h"

struct RocketSystems {
    Sensors sensors;
    RocketData rocket_data;
    LogSink& log_sink;
    BuzzerController buzzer;
    Telemetry tlm;
};

void begin_systems(RocketSystems* config);
