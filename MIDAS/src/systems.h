#pragma once

#include "sensors.h"
#include "rocket_state.h"
#include "data_logging.h"
#include "./hardware/telemetry.h"


struct RocketSystems {
    Sensors sensors;
    RocketData rocket_data;
    LogSink log_sink;
    Telemetry tlm;
};

void begin_systems(RocketSystems* config);
