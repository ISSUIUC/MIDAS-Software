#pragma once

#include <FS.h>

#include <SD.h>
#include "sensors.h"
#include "data_logging.h"

/**
 * @class SDSink
 * 
 * @brief Class that wraps the SD card functions
*/
class SDSink : public LogSink {
public:
    bool failed = false;

    SDSink() = default;

    ErrorCode init() override;
    void write(const uint8_t* data, size_t size) override;
    void write_meta(const uint8_t* data, size_t size) override;
private:
    File file;
    File meta;
    size_t unflushed_bytes = 0;
};
