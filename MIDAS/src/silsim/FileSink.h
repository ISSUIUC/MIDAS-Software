#pragma once

#include "errors.h"
#include <fstream>
#include <cstdint>
#include "data_logging.h"

struct SDSink final: ILogSink {
    explicit SDSink(const char* file_name);

    ErrorCode init() override;
    void write(const uint8_t* data, size_t size) override;

private:
    std::ofstream* output_file;
};
