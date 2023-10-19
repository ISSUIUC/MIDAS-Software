#pragma once

#include <FS.h>

#include "sensors.h"

struct LogSink {
    ErrorCode init();
    void write(const uint8_t* data, size_t size);

    File file;
};