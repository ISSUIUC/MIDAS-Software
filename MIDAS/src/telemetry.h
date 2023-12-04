#pragma once

#include "telemetry_packet.h"
#include "rocket_state.h"

#if defined(SILSIM)
#include "silsim/telemetry.h"
#elif defined(HILSIM)
#else
#include "hardware/telemetry_backend.h"
#endif

class Telemetry {
public:
    Telemetry() = default;
    ErrorCode __attribute__((warn_unused_result)) init();

    void transmit(RocketData& rocket_data);
    void bufferData(RocketData& rocket_data);

private:
    Queue<TelemetryDataLite, 4> small_packet_queue;

    // Initializing command ID
    int16_t last_command_id = -1;

    float set_frequency_to = NAN;
    char callsign[8] = "NO SIGN";

    TelemetryPacket makePacket(RocketData& data);
    void handleCommand(const telemetry_command& cmd);

    TelemetryBackend backend;
};
