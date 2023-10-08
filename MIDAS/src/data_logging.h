#pragma once

#include "rocket_state.h"

#if defined(SILSIM)
#elif defined(HILSIM)
#else
#include "hardware/SDLog.h"
#endif

/**
 * Protocol for a sink, which is implemented as an SD card in hardware.
 */

// struct LogSink {
//     ErrorCode init();
//     void write(const uint8_t* data, size_t size);
// };

void log_data(LogSink& sink, RocketData& data);