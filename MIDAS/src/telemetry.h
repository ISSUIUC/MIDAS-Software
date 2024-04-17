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
private:
    TelemetryPacket makePacket(RocketData& data);

    TelemetryBackend backend;
};
