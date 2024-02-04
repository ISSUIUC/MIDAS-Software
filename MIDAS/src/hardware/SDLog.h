#pragma once

#include <FS.h>

#include "sensors.h"
#include "data_logging.h"

class LogSink : public Logger {
    public:
        LogSink() = default;

        ErrorCode init();
        void write(const uint8_t* data, size_t size);
};
