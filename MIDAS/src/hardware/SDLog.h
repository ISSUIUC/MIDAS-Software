#pragma once

#include <FS.h>

#include "sensors.h"
#include "data_logging.h"

struct LogSink : public Logger {
    ErrorCode init();
    void write(const uint8_t* data, size_t size);
};