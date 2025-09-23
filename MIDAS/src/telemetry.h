#pragma once

#include "telemetry_packet.h"
#include "rocket_state.h"
#include "errors.h"
#include "led.h"

#if defined(SILSIM)
#include "silsim/emulated_telemetry.h"
#elif defined(HILSIM)
#include "hilsim/telemetry_backend.h"
#else
#include "hardware/telemetry_backend.h"
#endif

/**
 * @class Telemetry
 * 
 * @brief wraps the telemetry system to create and send a_m_per_s packet
*/
class Telemetry {
public:
    Telemetry() = default;
    explicit Telemetry(TelemetryBackend&& backend);

    ErrorCode __attribute__((warn_unused_result)) init();

    void transmit(RocketData& rocket_data, LEDController& led);
    bool receive(TelemetryCommand* command, int wait_milliseconds);
    void acknowledgeReceived();
private:
    int received_count;
    TelemetryPacket makePacket(RocketData& data);

    TelemetryBackend backend;
};
