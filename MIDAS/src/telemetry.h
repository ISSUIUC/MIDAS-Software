#pragma once

#include "hardware_interface.h"
#include "telemetry_packet.h"
#include "rocket_state.h"
#include "led.h"


/**
 * @class Telemetry
 * 
 * @brief wraps the telemetry system to create and send a packet
*/
class Telemetry {
public:
    explicit Telemetry(ITelemetryBackend& backend);

    ErrorCode init();

    void transmit(RocketData& rocket_data, LEDController& led);
    bool receive(TelemetryCommand* command, int wait_milliseconds);
    void acknowledgeReceived();

private:
    ITelemetryBackend& backend;
    int received_count = 0;

    TelemetryPacket makePacket(RocketData& data) const;
};
