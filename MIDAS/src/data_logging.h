#pragma once

#include "rocket_state.h"

#if defined(SILSIM)
#include "silsim/FileSink.h"
#elif defined(HILSIM)
#else
#include "hardware/SDLog.h"
#include "hardware/Emmc.h"
#endif

/**
 * Protocol for a sink, which is implemented as an SD card in hardware.
 */

// struct LogSink {
//     ErrorCode init();
//     void write(const uint8_t* data, size_t size);
// };

struct Logger {
    virtual ErrorCode init() = 0;
    virtual void write(const uint8_t* data, size_t size) = 0;

    File file;
}

void log_begin(LogSink& sink);
void log_data(LogSink& sink, RocketData& data);