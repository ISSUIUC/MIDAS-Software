#pragma once

#include <fstream>
#include <hardware_interface.h>

#include "errors.h"

class TelemetryBackend final: public ITelemetryBackend {
public:
    explicit TelemetryBackend(const char* file_name);
    ErrorCode init() override;

    int8_t getRecentRssi() override;
    void setFrequency(float frequency) override;

protected:
    void send_bytes(const uint8_t* data, size_t length) override;

    bool recv_bytes(uint8_t* data, size_t length, int wait_milliseconds) override;

private:
    std::ofstream output_file;
};
