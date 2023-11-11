#include <Arduino.h>
#include <pb_decode.h>

#include "hilsim/hilsimpacket.pb.h"

void setup() {
    uint8_t buffer[HILSIMPacket_size];
    // Read the thing
    // Serial???
    while (true) {
        // No way the packet size goes over 128, right?
        uint8_t length = Serial.read();
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
        char out[128];
        sprintf(out, "%f\n", packet.barometer_pressure);
        Serial.write(out, strlen(out));
    }
}

void loop(){}
