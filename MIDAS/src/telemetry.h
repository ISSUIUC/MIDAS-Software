#pragma once

#include "telemetry_packet.h"
#include "rocket_state.h"
#include "errors.h"
#include "led.h"
#include "esp_eeprom.h"

#if defined(SILSIM)
#include "silsim/emulated_telemetry.h"
#elif defined(HILSIM)
#include "hilsim/sensors/telemetry_backend.h"
#else
#include "hardware/telemetry_backend.h"
#endif

/**
 * @class Telemetry
 * 
 * @brief wraps the telemetry system to create and send a packet
*/
class Telemetry {
public:
    Telemetry() = default;
    explicit Telemetry(TelemetryBackend&& backend);

    ErrorCode __attribute__((warn_unused_result)) init();

    void transmit(RocketData& rocket_data, const MIDASEEPROM& eeprom, LEDController& led);
    bool receive(TelemetryCommand* command, int wait_milliseconds);
    void acknowledgeReceived();
    ErrorCode setFrequency(float frequency);
    void set_spi_mutex(SemaphoreHandle_t mtx) { backend.set_spi_mutex(mtx); }
private:
    int received_count;
    TelemetryPacket makePacket(RocketData& data, const MIDASEEPROM& eeprom);

    TelemetryBackend backend;
};
