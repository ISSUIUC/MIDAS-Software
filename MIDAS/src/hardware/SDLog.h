#pragma once

#include <FS.h>

#include <SD.h>
#include "sensors.h"
#include "data_logging.h"

class SDSink : public LogSink {
public:
    bool failed = false;

    SDSink() = default;

    ErrorCode init() override;
    void write(const uint8_t* data, size_t size) override;
private:
    File file;
    size_t unflushed_bytes = 0;
};
