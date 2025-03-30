#pragma once

#include "hal.h"
#include "telemetry_packet.h"
#include "rocket_state.h"
#include "led.h"

TelemetryPacket makePacket(RocketData& data, int received_count);

/**
 * @class Telemetry
 * 
 * @brief wraps the telemetry system to create and send a packet
 */
template<typename Hw>
class Telemetry {
public:
    Telemetry(HwInterface<Hw>& hw, LEDController<Hw>& led) : hw(hw), led(led) { }

    void transmit(RocketData& rocket_data) {
        static_assert(sizeof(TelemetryPacket) <= 0xFF, "The data type to send is too large"); // Max payload is 255

        TelemetryPacket packet = makePacket(rocket_data, received_count);
        led.toggle(LED::BLUE);

        hw.transmit(&packet);
    }

    bool receive(TelemetryCommand* command, int wait_milliseconds) {
        static_assert(sizeof(TelemetryCommand) <= 0xFF, "The data type to receive is too large"); // Max payload is 255
        return hw.receive(command, wait_milliseconds);
    }

    void acknowledgeReceived() {
        received_count++;
    }

private:
    int received_count = 0;

    HwInterface<Hw>& hw;
    LEDController<Hw>& led;
};
