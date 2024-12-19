#pragma once

#include <fstream>

#include "errors.h"
#include "hal.h"


class TelemetryBackend {
public:
    TelemetryBackend() = default;
    explicit TelemetryBackend(const char* file_name);
    ErrorCode __attribute__((warn_unused_result)) init();

    int8_t getRecentRssi();
    void setFrequency(float frequency);

    template<typename T>
    void send(const T& data) {
        output_file.write((const char*) &data, sizeof(T));
    }

    template<typename T>
    bool read(T* write, int wait_milliseconds) {
        return false;
    }

private:
    std::ofstream output_file;
};