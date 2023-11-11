#include "hilsim.h"

#include <Arduino.h>
#include <pb_decode.h>
#include <pb_decode.h>

#include "hilsim/hilsimpacket.pb.h"

void hilsim() {
    char buffer[HILSIMPacket_size];
    // Read the thing
    // Serial???
    while (true) {
        // No way the packet size goes over 128, right?
        char length = Serial.read();
        // Parse the two bytes as integers
        size_t size = Serial.readBytes(buffer, length);
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
        // Write data
    }
}
