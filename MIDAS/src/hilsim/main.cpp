#include <Arduino.h>
#include <pb_decode.h>

#include <systems.h>
#include "packet.h"

HILSIMPacket global_packet = HILSIMPacket_init_zero;
pb_byte_t buffer[HILSIMPacket_size];

RocketSystems systems;

DECLARE_THREAD(hilsim, void*arg){
    uint8_t buffer[HILSIMPacket_size];
    Serial.begin(9600);

    while (true) {
        // No way the packet size goes over 128, right?
        uint8_t length = Serial.read();
        // Parse the two bytes as integers
        Serial.println("Reading packet");
        size_t size = Serial.readBytes(buffer, length);
        // Kill off null zeros?
        HILSIMPacket packet = HILSIMPacket_init_zero;
        pb_istream_t stream = pb_istream_from_buffer(buffer, size);
        bool status = pb_decode(&stream, HILSIMPacket_fields, &packet);
        if (!status) {
            // Error
            Serial.println("Error reading packet");
        }
        // Process information
        // Loop
        // Write data
        char out[128];
        sprintf(out, "%f\n", packet.barometer_pressure);
        Serial.println(out);
        global_packet = packet;
        THREAD_SLEEP(10);
    }
}

void setup() {
    Serial.begin(9600);
    while(true){
        Serial.println("HI");
    }
    START_THREAD(hilsim, 1, nullptr);
    begin_systems(&systems);
    
}

void loop(){}
