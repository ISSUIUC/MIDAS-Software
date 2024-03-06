/**
 * @file telemetry.cpp
 *
 * @brief This file defines the telemetry class used to facilitate
 * telemetry commands and data transfer between the on-board flight
 * computer and the ground station.
 *
 * Spaceshot Avionics 2023-24
 * Illinois Space Society - Software Team
 * Gautam Dayal
 * Nicholas Phillips
 * Patrick Marschoun
 * Peter Giannetos
 * Rishi Gokkumutkkala
 * Aaditya Voruganti
 * Magilan Sendhil
 */

#include <RHHardwareSPI.h>

#include "hardware/telemetry_backend.h"
#include "hardware/pins.h"


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

TelemetryBackend::TelemetryBackend() : rf95(TELEMETRY_CS, TELEMETRY_INT, hardware_spi) {
    led_state = false;
    led_pin = LED_BLUE;
}

ErrorCode TelemetryBackend::init() {
    pinMode(TELEMETRY_RESET, OUTPUT);
    digitalWrite(TELEMETRY_RESET, HIGH);
    delay(10);

    digitalWrite(TELEMETRY_RESET, LOW);
    delay(10);
    digitalWrite(TELEMETRY_RESET, HIGH);
    delay(10);

    if (!rf95.init()) {
        return ErrorCode::RadioInitFailed;
    }
    Serial.println("[DEBUG]: Radio Initialized");

    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf =
    // 128chips/symbol, CRC on

    if (!rf95.setFrequency(RF95_FREQ)) {
        return ErrorCode::RadioSetFrequencyFailed;
    }

    /*
     * The default transmitter power is 13dBm, using PA_BOOST.
     * If you are using RFM95/96/97/98 modules which uses the PA_BOOST
     * transmitter pin, then you can set transmitter powers from 5 to 23 dBm:
     */
    rf95.setTxPower(6, false);

    sei();
    return ErrorCode::NoError;
}

int8_t TelemetryBackend::getRecentRssi() {
    return rf95.lastRssi();
}

void TelemetryBackend::setFrequency(float freq) {
    rf95.setFrequency(freq);
}
