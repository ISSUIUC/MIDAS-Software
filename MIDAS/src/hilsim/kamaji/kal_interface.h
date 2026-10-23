#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "TCAL9538.h"
#include <flight-systems/systems.h>
// Handles MIDAS system setup and interfacing

inline void k_midas_setup() {

    delay(200);

    // Immediate startup tone
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);

    ledcWriteTone(BUZZER_CHANNEL, 3200);
    delay(250);
    ledcWriteTone(BUZZER_CHANNEL, 0);

    // begin sensor SPI bus
    Serial.println("Starting SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    //begin I2C bus (for the GPIO expander that drives pyros)
    Serial.println("Starting I2C...");
    Wire.begin(I2C_SDA, I2C_SCL, 100000);

    if (!TCAL9538Init(EXP_RESET)) {
        Serial.println(":(");
    }

    //set chip selects high (deselected)
    pinMode(E22_CS, OUTPUT);
    pinMode(MS5611_CS, OUTPUT);
    pinMode(IMU_CS_PIN, OUTPUT);
    pinMode(MMC5983_CS, OUTPUT);

    digitalWrite(MS5611_CS, HIGH);
    digitalWrite(E22_CS, HIGH);
    digitalWrite(IMU_CS_PIN, HIGH);
    digitalWrite(MMC5983_CS, HIGH);

    // b2b pins
    pinMode(B2B_EN, OUTPUT);
    pinMode(B2B_READY, INPUT);
    digitalWrite(B2B_EN, HIGH);

    //configure output leds (direct ESP32 pins on current hardware)
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);
    pinMode(LED_RED, OUTPUT);

    // pyros (on the GPIO expander)
    for (int i = 0; i < MIDAS_NUM_PYROS; i++) {
        gpioPinMode(PYRO_PINS[i], OUTPUT);
    }
    gpioPinMode(PYRO_GLOBAL_ARM_PIN, OUTPUT);

    delay(200);
}

inline void k_ident() {
    Serial.println("%id:MIDAS");
}