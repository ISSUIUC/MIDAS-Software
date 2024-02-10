#include <ACAN2517.h>
#include <SPI.h>

#include "pins.h"
#include "hal.h"

// idk if we'll ever test this so no need to stuff it in a box

#define MESSAGE_ID 0x542

#define CAMERA_DEBUG

ACAN2517 can(MCP2517_CS, SPI, MCP2517_INT);

void can_setup() {
    ACAN2517Settings settings(ACAN2517Settings::OSC_4MHz10xPLL, CAN_BIT_RATE); // CAN bit rate 125 kb/s

#ifdef CAMERA_DEBUG
    settings.mRequestedMode = ACAN2517Settings::InternalLoopBack; // Select loopback mode
#endif

    const uint32_t errorCode = can.begin(settings, [] { can.isr(); });
    if (errorCode != 0) {
        Serial.print("Configuration error 0x");
        Serial.println(errorCode, HEX);
    }
}

void can_loop(bool should_turn_cam_on) {
    CANMessage message;
    message.id = MESSAGE_ID;
    message.data[0] = should_turn_cam_on;

    while (!can.tryToSend(message)) {
        // keep trying until it's sent
        // idk what else we can do
        THREAD_SLEEP(500);
    }
    THREAD_SLEEP(2000);
}