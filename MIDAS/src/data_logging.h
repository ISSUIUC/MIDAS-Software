#pragma once

#include "errors.h"
#include "rocket_state.h"

/**
 * Protocol for a sink, which is implemented as an SD card in hardware.
 */
struct LogSink {
    ErrorCode init();
    void write(const uint8_t* data, size_t size);
};

void log_data(LogSink& sink, RocketData& data);