#pragma once

#include <fstream>

#include "errors.h"

class TelemetryBackend {
public:
    explicit TelemetryBackend(const char* file_name);
    ErrorCode init();

    int8_t getRecentRssi();
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
