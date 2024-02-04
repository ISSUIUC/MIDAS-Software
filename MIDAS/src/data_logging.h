#pragma once

#include "rocket_state.h"

#if defined(SILSIM)
#include "silsim/FileSink.h"
#elif defined(HILSIM)
#else
// #include "hardware/SDLog.h"
// #include "hardware/Emmc.h"
#endif

/**
 * Protocol for a sink, which is implemented as an SD card in hardware.
 */

// struct LogSink {
//     ErrorCode init();
//     void write(const uint8_t* data, size_t size);
// };

class Logger {
    public:
        Logger() = default;

        virtual ErrorCode init() {return ErrorCode::NoError;};
        virtual void write(const uint8_t* data, size_t size) {return;};

        File file;
};

void log_begin(Logger& sink);
void log_data(Logger& sink, RocketData& data);