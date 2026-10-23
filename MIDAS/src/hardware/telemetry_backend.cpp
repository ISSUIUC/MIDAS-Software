/**
 * @file telemetry.cpp
 *
 * @brief Implements the telemetry backend responsible for LoRa
 * communication between the flight computer and ground station.
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
#define TX_FREQ 421.15

#define TX_OUTPUT_POWER 22          // dBm
#define LORA_BANDWIDTH 0            // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 8     // [SF7..SF12]
#define LORA_CODINGRATE 4           // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH 10     // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0       // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 1000
#define TX_TIMEOUT_VALUE 1000
#define LORA_BUFFER_SIZE 64         // Define the payload size here

TelemetryBackend::TelemetryBackend()
    : lora(SPI, E22_CS, E22_BUSY, E22_DI01, E22_RXEN, E22_RESET) {

    // Construct the SX1268 driver with the board-specific SPI and GPIO
    // connections.
}

ErrorCode TelemetryBackend::init() {

    // Initialize communication with the LoRa transceiver.
    if (lora.setup() != SX1268Error::NoError)
        return ErrorCode::LoraCouldNotBeInitialized;

    // Configure modulation parameters (spreading factor, bandwidth,
    // coding rate, and header mode).
    if (lora.set_modulation_params(8, LORA_BW_250, LORA_CR_4_8, false) != SX1268Error::NoError)
        return ErrorCode::LoraCommunicationFailed;

    // Configure the operating RF frequency.
    if (lora.set_frequency((uint32_t)(TX_FREQ * 1e6)) != SX1268Error::NoError)
        return ErrorCode::LoraCommunicationFailed;

    // Configure transmit output power.
    if (lora.set_tx_power(22) != SX1268Error::NoError)
        return ErrorCode::LoraCommunicationFailed;

    return ErrorCode::NoError;
}

int16_t TelemetryBackend::getRecentRssi() {

    // RSSI reporting is currently not implemented by the radio driver.
    return 0;
}

ErrorCode TelemetryBackend::setFrequency(float freq) {

    // Update the radio operating frequency.
    if (lora.set_frequency((uint32_t)(freq * 1e6)) != SX1268Error::NoError) {
        return ErrorCode::LoraCommunicationFailed;
    }

    return ErrorCode::NoError;
}