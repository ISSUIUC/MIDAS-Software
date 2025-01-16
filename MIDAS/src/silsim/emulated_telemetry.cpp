#include "emulated_telemetry.h"


TelemetryBackend::TelemetryBackend(const char* file_name) {
    output_file.open(file_name, std::ios::out | std::ios::binary | std::ios::trunc);
}

ErrorCode __attribute__((warn_unused_result)) TelemetryBackend::init() {
    return ErrorCode::NoError;
}

int8_t TelemetryBackend::getRecentRssi() {
    return 1;
}

void TelemetryBackend::setFrequency(float frequency) {
    (void) frequency;
}

void TelemetryBackend::send_bytes(const uint8_t* data, size_t length) {
    output_file.write((const char*) data, length);
}

bool TelemetryBackend::recv_bytes(uint8_t* data, size_t length, int wait_milliseconds) {
    return false;
}
