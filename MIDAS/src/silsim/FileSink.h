#pragma once

#include "errors.h"
#include <fstream>
#include <cstdint>
#include "data_logging.h"

struct FileSink : public LogSink {
    explicit FileSink(const char* file_name);

    ErrorCode init();
    void write(const uint8_t* data, size_t size);

private:
    std::ofstream output_file;
};