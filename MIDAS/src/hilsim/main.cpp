#include <Arduino.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include <systems.h>
#include "global_packet.h"
#include "git_hash.h"

HILSIMPacket global_packet = HILSIMPacket_init_zero;
pb_byte_t buffer[HILSIMPacket_size];

RocketSystems systems;

DECLARE_THREAD(hilsim, void*arg) {
    uint8_t buffer[HILSIMPacket_size];
    int n = 0;
    // Debug kamaji output to verify if we're reading the correct packets
    while (Serial.read() != 33);
    char magic[] = {69, 110, 117, 109, 99, 108, 97, 119, 0};
    Serial.println(magic);
    Serial.println(GIT_HASH_STRING);
    Serial.println(__TIME__);
    Serial.println(__DATE__);
    Serial.flush();

    while (true) {
        if(!Serial.available()){
            THREAD_SLEEP(1);
            continue;
        }
        uint16_t length = ((uint16_t) Serial.read()) << 8 + Serial.read();
        // Parse the two bytes as integers
        size_t hilsim_packet_size = Serial.readBytes(buffer, length);
        HILSIMPacket packet = HILSIMPacket_init_zero;
        pb_istream_t stream = pb_istream_from_buffer(buffer, hilsim_packet_size);
        bool status = pb_decode(&stream, HILSIMPacket_fields, &packet);
        if (!status) {
            Serial.flush();
            THREAD_SLEEP(10);
            continue;
        }
        RocketState rocket_state = RocketState_init_zero;
        rocket_state.rocket_state = n;
        uint8_t buffer[RocketState_size];
        pb_ostream_t output_stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        status = pb_encode(&output_stream, RocketState_fields, &rocket_state);
        Serial.write(output_stream.bytes_written);
        Serial.write(buffer, output_stream.bytes_written);
        Serial.flush();
        global_packet = packet;
        n++;
        THREAD_SLEEP(10);
    }
}

void setup() {
    Serial.begin(9600);
    while (!Serial);
    begin_hilsim(&systems);
    hilsim_thread(nullptr);
}

void loop(){}
