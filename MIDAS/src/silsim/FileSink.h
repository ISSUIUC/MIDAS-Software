#pragma once

#include "errors.h"
#include <fstream>

struct LogSink {
    ErrorCode init();
    void write(const uint8_t* data, size_t size);

    std::ofstream output_file;
};