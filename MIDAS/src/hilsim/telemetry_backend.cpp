#include "telemetry_backend.h"


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