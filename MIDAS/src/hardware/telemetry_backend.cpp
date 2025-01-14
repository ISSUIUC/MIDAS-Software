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

#include <FreeRTOS.h>
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
TelemetryBackend::TelemetryBackend() : rf95(RFM96_CS, RFM96_INT) { }

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
    rf95.setSignalBandwidth(125000);
    rf95.setCodingRate4(8);
    rf95.setSpreadingFactor(10);
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

void TelemetryBackend::send_bytes(const uint8_t* data, size_t length) {
    //TODO assert(length <= RH_RF95_MAX_MESSAGE_LEN, "The data type to send is too large");
    rf95.send(data, length);
    for(int i = 1;; i++){
        THREAD_SLEEP(1);
        if(digitalRead(rf95._interruptPin)){
            break;
        }
        if(i % 1024 == 0){
            Serial.println("long telem wait");
        }
    }
    rf95.handleInterrupt();
}

bool TelemetryBackend::recv_bytes(uint8_t* data, size_t length, int wait_milliseconds) {
    //TODO assert(length <= RH_RF95_MAX_MESSAGE_LEN, "The data type to receive is too large");
    uint8_t len = length;

    // set receive mode
    rf95.setModeRx();

    // busy wait for interrupt signalling
    for(int i = 1; i < wait_milliseconds; i++){
        THREAD_SLEEP(1);
        if(digitalRead(rf95._interruptPin)){
            rf95.handleInterrupt();
            break;
        }
    }

    if (rf95.available() && rf95.recv(data, &len)) {
        return length == len;
    }
    return false;
}
