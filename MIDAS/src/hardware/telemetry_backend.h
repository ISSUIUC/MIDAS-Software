#pragma once

#include <RH_RF95.h>

#include "hardware_interface.h"

/**
 * @class TelemetryBackend
 * 
 * @brief Class that wraps the Telemetry functions
*/
class TelemetryBackend final: public ITelemetryBackend {
public:
    TelemetryBackend();
    ErrorCode init() override;

    int8_t getRecentRssi() override;
    void setFrequency(float frequency) override;

protected:
    void send_bytes(const uint8_t* data, size_t length) override;
    bool recv_bytes(uint8_t* data, size_t length, int wait_milliseconds) override;

private:
    RH_RF95 rf95;
};
