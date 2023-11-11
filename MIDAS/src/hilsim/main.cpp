#include <Arduino.h>
#include <pb_decode.h>

#include <systems.h>
#include "packet.h"

HILSIMPacket global_packet = HILSIMPacket_init_zero;
pb_byte_t buffer[HILSIMPacket_size];

RocketSystems systems;

DECLARE_THREAD(hilsim, void*arg){
    while (true) {
        size_t size = Serial.readBytes(buffer, HILSIMPacket_size + 1);
        // Kill off null zeros?
        HILSIMPacket packet = HILSIMPacket_init_zero;
        pb_istream_t stream = pb_istream_from_buffer(buffer, size);
        bool status = pb_decode(&stream, HILSIMPacket_fields, &packet);
        if (!status) {
            // Error
        }
        global_packet = packet;
        THREAD_SLEEP(10);
    }
}

void setup() {
    START_THREAD(hilsim, 1, nullptr);
    begin_systems(&systems);
    
}

void loop(){}
