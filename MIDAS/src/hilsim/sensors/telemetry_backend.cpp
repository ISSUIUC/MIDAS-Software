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

#include "hardware/telemetry_backend.h"
#include "hardware/pins.h"

// Change to 434.0 or other frequency, must match RX's freq!
#ifdef IS_BOOSTER
#define TX_FREQ 425.15
#else
#define TX_FREQ 421.15
#endif

#define TX_OUTPUT_POWER 22		// dBm
#define LORA_BANDWIDTH 0		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 8// [SF7..SF12]
#define LORA_CODINGRATE 4		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 10  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 1000
#define TX_TIMEOUT_VALUE 1000
#define LORA_BUFFER_SIZE 64 // Define the payload size here

/**
 * @brief Default constructor for the telemetry system
*/
TelemetryBackend::TelemetryBackend() : lora(SPI, E22_CS, E22_BUSY, E22_DI01, E22_RXEN, E22_RESET) {
    led_state = false;
}
/**
 * @brief Initializes the telemetry system
 * 
 * @return Error Code
*/
ErrorCode TelemetryBackend::init() {
	if(lora.setup() != SX1268Error::NoError) return ErrorCode::LoraCouldNotBeInitialized;
    if(lora.set_modulation_params(8, LORA_BW_250, LORA_CR_4_8, false) != SX1268Error::NoError) return ErrorCode::LoraCommunicationFailed;
	if(lora.set_frequency((uint32_t) (TX_FREQ * 1e6)) != SX1268Error::NoError) return ErrorCode::LoraCommunicationFailed;
	if(lora.set_tx_power(22) != SX1268Error::NoError) return ErrorCode::LoraCommunicationFailed;

    return ErrorCode::NoError;
}

/**
 * @brief Gets RSSI of recent packets
 * 
 * @return RSSI of most recent packet
*/
int16_t TelemetryBackend::getRecentRssi() {
    return 0;
}

/**
 * @brief Sets new frequency for the LoRa module
 *
 * @param freq New frequency to set the LoRa module to
*/
ErrorCode TelemetryBackend::setFrequency(float freq) {
    if(lora.set_frequency((uint32_t) (freq * 1e6)) != SX1268Error::NoError) {
        return ErrorCode::LoraCommunicationFailed;
    } else {
        return ErrorCode::NoError;
    }
}
