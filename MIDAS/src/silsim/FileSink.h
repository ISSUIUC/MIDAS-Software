#pragma once

#include "errors.h"
#include <fstream>

struct LogSink {
    explicit LogSink(const char* file_name);

    ErrorCode init();
    void write(const uint8_t* data, size_t size);

private:
    std::ofstream output_file;
};