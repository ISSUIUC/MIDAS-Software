/**
 * This file contains the code than runs on our ground station
 * hardware (LoRa Feather module). It includes the receive code,
 * command functionality, and interfaces with the ground station GUI
 * through serial.
 *
 * Spaceshot Telemetry Team 2023-24
 * Nicholas Phillips
 * Gautam Dayal
 * Patrick Marschoun
 * Peter Giannetos
 * Aaditya Voruganti
 * Micheal Karpov
 */

#include <RH_RF95.h>
#include <SPI.h>

#include <array>
#include <limits>
#include <numeric>
#include <queue>
#include <algorithm>

/* Pins for feather*/
// // Ensure to change depending on wiring
#define RFM95_CS 8
#define RFM95_RST 4
// #define RFM95_EN
#define RFM95_INT 3
#define VoltagePin 14
// #define LED 13 // Blinks on receipt

float RF95_FREQ = 420;
float SUSTAINER_FREQ = 426.15;
float BOOSTER_FREQ = 425.15;
float GROUND_FREQ = 420;

float current_freq = 0;

#define DEFAULT_CMD 0
#define MAX_CMD_LEN 10

typedef uint32_t systime_t;
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
// RH_RF95 rf95(RFM95_CS, RFM95_INT);

SerialParser serial_parser(SerialInput, SerialError);

void setup() {
    while (!Serial)
        ;
    Serial.begin(9600);
    if (!rf95.init()) {
        Serial.println(json_init_failure);
        while (1);
    }
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println(json_init_success);
    rf95.setCodingRate4(8);
    rf95.setSpreadingFactor(10);
    rf95.setPayloadCRC(true);
    rf95.setSignalBandwidth(125000);
    Serial.print(R"({"type": "freq_success", "frequency":)");
    Serial.print(current_freq);
    Serial.println("}");
    rf95.setTxPower(23, false);
}

void loop() {
    
    if (rf95.available()) {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        TelemetryPacket packet;
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(50);
            digitalWrite(LED_BUILTIN, LOW);
            // Serial.println("Received packet");
            // Serial.println(len);
            memcpy(&packet, buf, sizeof(packet));
            EnqueuePacket(packet, current_freq);
            if (!cmd_queue.empty()) {
                auto& cmd = cmd_queue.front();
                    cmd.retry_count++;
                    if (cmd.retry_count >= max_command_retries) {
                        cmd_queue.pop();
                        Serial.println(json_send_failure);
                    }
            }

            process_command_queue();

        } else {
            Serial.println(json_receive_failure);
        }
    }
}