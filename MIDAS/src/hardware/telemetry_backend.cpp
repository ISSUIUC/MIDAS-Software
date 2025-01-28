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
#ifdef IS_BOOSTER
#define RF95_FREQ 425.15
#else
#define RF95_FREQ 426.15
#endif

/**
 * @brief Default constructor for the telemetry system
*/
TelemetryBackend::TelemetryBackend() : rf95(RFM96_CS, RFM96_INT) {
    led_state = false;
}

/**
 * @brief Initializes the telemetry system
 * 
 * @return Error Code
*/
ErrorCode TelemetryBackend::init() {
    pinMode(RFM96_RESET, OUTPUT);
    digitalWrite(RFM96_RESET, HIGH);
    delay(100);
    digitalWrite(RFM96_RESET, LOW);
    delay(100);
    digitalWrite(RFM96_RESET, HIGH);
    delay(5);

    if (!rf95.init()) {
        return ErrorCode::RadioInitFailed;
    }
    Serial.println("[DEBUG]: Radio Initialized");

    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf =
    // 128chips/symbol, CRC on

    if (!rf95.setFrequency(RF95_FREQ)) {
        return ErrorCode::RadioSetFrequencyFailed;
    }
    rf95.setSignalBandwidth(250000);
    rf95.setCodingRate4(6);
    rf95.setSpreadingFactor(8);
    rf95.setPayloadCRC(true);
    /*
     * The default transmitter power is 13dBm, using PA_BOOST.
     * If you are using RFM95/96/97/98 modules which uses the PA_BOOST
     * transmitter pin, then you can set transmitter powers from 5 to 23 dBm:
     */
    rf95.setTxPower(23, false);

    sei();
    return ErrorCode::NoError;
}

/**
 * @brief Gets RSSI of recent packets
 * 
 * @return RSSI of most recent packet
*/
int8_t TelemetryBackend::getRecentRssi() {
    return rf95.lastRssi();
}

/**
 * @brief Sets new frequency for the LoRa module
 * 
 * @param freq New frequency to set the LoRa module to
*/
void TelemetryBackend::setFrequency(float freq) {
    rf95.setFrequency(freq);
}
