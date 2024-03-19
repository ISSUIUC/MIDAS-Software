#pragma once

#include <FS.h>

#include <SD.h>
#include "sensors.h"
#include "data_logging.h"

class FileSink : public LogSink {
public:
    FileSink() = default;

    ErrorCode init() override;
    void write(const uint8_t* data, size_t size) override;
private:
    File file;
};
