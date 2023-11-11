#include <Arduino.h>
#include <pb_decode.h>

#include "hilsim/hilsimpacket.pb.h"

void setup() {
    pb_byte_t buffer[HILSIMPacket_size];
    // Read the thing
    // Serial???
    while (true) {
        size_t size = Serial.readBytes(buffer, HILSIMPacket_size + 1);
        // Kill off null zeros?
        HILSIMPacket packet = HILSIMPacket_init_zero;
        pb_istream_t stream = pb_istream_from_buffer(buffer, size);
        bool status = pb_decode(&stream, HILSIMPacket_fields, &packet);
        if (!status) {
            // Error
            return;
        }
        // Process information
        // Loop
        // Assign values to hilsim and do stuff
    }
}

void loop(){}
