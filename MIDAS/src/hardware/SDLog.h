#pragma once

#include <FS.h>
#include "hardware_interface.h"

/**
 * @class SDSink
 * 
 * @brief Class that wraps the SD card functions
*/
class SDSink : public ILogSink {
public:
    bool failed = false;

    SDSink() = default;

    ErrorCode init() override;
    void write(const uint8_t* data, size_t size) override;
private:
    File file;
    size_t unflushed_bytes = 0;
};

/**
 * @class EMMCSink
 *
 * @brief Class that wraps the emmc functions
*/
class EMMCSink : public ILogSink {
public:
    EMMCSink() = default;

    ErrorCode init() override;
    void write(const uint8_t* data, size_t size) override;
private:
    File file;
    size_t unflushed_bytes = 0;
};
