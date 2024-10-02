#include <Arduino.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include "global_packet.h"

HILSIMPacket global_packet = HILSIMPacket_init_zero;

MultipleLogSink<> sink;
RocketSystems systems{.log_sink = sink};

DECLARE_THREAD(hilsim, void*arg) {
    uint8_t buffer[HILSIMPacket_size];
    int n = 0;
    // Debug kamaji output to verify if we're reading the correct packets
    while (Serial.read() != 33);
    char magic[] = {69, 110, 117, 109, 99, 108, 97, 119, 0};
    Serial.println(magic);
    Serial.println(__TIME__);
    Serial.println(__DATE__);
    Serial.flush();

    while (true) {
        while (!Serial.available());
        uint8_t a = Serial.read();
        uint8_t b = Serial.read();
        uint16_t length = (uint16_t) b + (((uint16_t) a) << 8);
        // Parse the two bytes as integers

        size_t hilsim_packet_size = Serial.readBytes(buffer, length);
        // Serial.print(length);
        // Serial.print(" ");
        // Serial.printf("%d %d ", a, b);
        HILSIMPacket packet = HILSIMPacket_init_zero;
        pb_istream_t stream = pb_istream_from_buffer(buffer, hilsim_packet_size);
        bool status = pb_decode(&stream, HILSIMPacket_fields, &packet);
        if (!status) {
            THREAD_SLEEP(10);
            continue;
        }
        global_packet = packet;
        RocketState rocket_state = RocketState_init_zero;
        rocket_state.rocket_state = (int) (100 * sin((double)n / 360));
        uint8_t buffer2[RocketState_size];
        pb_ostream_t output_stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        status = pb_encode(&output_stream, RocketState_fields, &rocket_state);
        Serial.write(output_stream.bytes_written);
        Serial.write(buffer, output_stream.bytes_written);
        Serial.flush();
        n++;
        THREAD_SLEEP(10);
    }
}

void setup() {
    Serial.begin(9600);
    while (!Serial);
    hilsim_thread(nullptr);
}

void loop(){}
