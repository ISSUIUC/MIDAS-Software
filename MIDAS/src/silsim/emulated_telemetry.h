#pragma once

#include <fstream>

#include "errors.h"
#include "hal.h"


class TelemetryBackend {
public:
    explicit TelemetryBackend(const char* file_name);
    ErrorCode __attribute__((warn_unused_result)) init();

    int16_t getRecentRssi();
    void setFrequency(float frequency);

    template<typename T>
    void send(const T& data) {
        output_file.write((const char*) &data, sizeof(T));
    }

    template<typename T>
    bool read(T* write) {
        return false;
    }

private:
    std::ofstream output_file;
};