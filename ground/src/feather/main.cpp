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

void setup() {
    while (!Serial)
        ;
    Serial.begin(9600);
    if (!rf95.init()) {
        while (1);
    }
    pinMode(LED_BUILTIN, OUTPUT);
    rf95.setFrequency(428);
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
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    rf95.send(buf, 4); 
    rf95.waitPacketSent();
    Serial.println("sent!");
}