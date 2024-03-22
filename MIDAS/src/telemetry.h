#pragma once

#include "telemetry_packet.h"
#include "rocket_state.h"
#include "errors.h"
#include "led.h"

#if defined(SILSIM)
#include "silsim/emulated_telemetry.h"
#elif defined(HILSIM)
#else
#include "hardware/telemetry_backend.h"
#endif

class Telemetry {
public:
    Telemetry() = default;
    explicit Telemetry(TelemetryBackend&& backend);

    ErrorCode __attribute__((warn_unused_result)) init();

    void transmit(RocketData& rocket_data, LEDController& led);
    void bufferData(RocketData& rocket);

private:
    Queue<TelemetryDataLite, 4> small_packet_queue;

    // Initializing command ID
    int16_t last_command_id = -1;

    float set_frequency_to = NAN;

    #ifdef IS_SUSTAINER
        char callsign[8] = "KD9ZMJ";
    #endif
    
    #ifdef IS_BOOSTER
        char callsign[8] = "KD9ZPM";
    #endif
    

    TelemetryPacket makePacket(RocketData& data);
    void handleCommand(const telemetry_command& cmd);

    TelemetryBackend backend;
};
