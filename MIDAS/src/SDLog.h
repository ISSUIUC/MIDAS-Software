#pragma once

#include <FS.h>

#include <SD.h>
#ifdef HILSIM
#include "hilsim/sensors.h"
#else
#include "hardware/sensors.h"
#endif
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
private:
    File file;
    size_t unflushed_bytes = 0;
};
