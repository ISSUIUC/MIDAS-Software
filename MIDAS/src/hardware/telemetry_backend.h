#pragma once

#include <util/errors.h>
#include "util/hal.h"
#include "pins.h"

#include <E22.h>

/**
 * @class TelemetryBackend
 *
 * @brief Interface for the onboard LoRa telemetry radio.
 *
 * Wraps the SX1268/E22 radio driver and provides functions for
 * initialization, packet transmission, reception, frequency control,
 * and SPI synchronization.
 */
class TelemetryBackend {
public:
    /**
     * @brief Constructs the telemetry backend.
     */
    TelemetryBackend();

    /**
     * @brief Initializes the LoRa radio.
     *
     * Configures the radio hardware and prepares it for packet
     * transmission and reception.
     *
     * @return Error code indicating initialization status.
     */
    [[nodiscard]] ErrorCode init();

    /**
     * @brief Returns the RSSI of the most recently received packet.
     *
     * @return Received Signal Strength Indicator (RSSI) in dBm.
     */
    int16_t getRecentRssi();

    /**
     * @brief Changes the operating radio frequency.
     *
     * @param frequency Desired frequency in MHz.
     *
     * @return Error code indicating whether the operation succeeded.
     */
    ErrorCode setFrequency(float frequency);

    /**
     * @brief Assigns the SPI mutex used by the radio driver.
     *
     * @param mtx FreeRTOS semaphore protecting the shared SPI bus.
     */
    void set_spi_mutex(SemaphoreHandle_t mtx) { lora.set_spi_mutex(mtx); }

    /**
     * @brief Transmits a packet over the LoRa radio.
     *
     * The packet type must fit within the SX1268 maximum payload size
     * of 255 bytes. If transmission fails, the radio is automatically
     * reinitialized.
     *
     * @tparam T Packet type to transmit.
     *
     * @param data Packet to send.
     */
    template<typename T>
    void send(const T& data) {
        static_assert(sizeof(T) <= 0xFF, "The data type to send is too large"); // Max payload is 255

        SX1268Error result = lora.send((uint8_t*) &data, sizeof(T));

        if(result != SX1268Error::NoError) {
            Serial.print("Lora TX error ");
            Serial.println((int)result);

            // Attempt to recover from communication failure.
            (void)init();
        }
    }

    /**
     * @brief Attempts to receive a packet from the LoRa radio.
     *
     * Waits up to the specified timeout for a packet of type T. Radio
     * errors automatically trigger a reinitialization attempt.
     *
     * @tparam T Packet type expected.
     *
     * @param write Buffer where the received packet will be stored.
     * @param wait_milliseconds Maximum receive timeout in milliseconds.
     *
     * @return true if a packet was successfully received.
     * @return false if the receive timed out or an error occurred.
     */
    template<typename T>
    bool read(T* write, int wait_milliseconds) {
        static_assert(sizeof(T) <= 0xFF, "The data type to receive is too large");

        uint8_t len = sizeof(T);

        // Receive a packet from the radio.
        SX1268Error result = lora.recv((uint8_t*) write, len, wait_milliseconds);

        if(result == SX1268Error::NoError) {
            return true;
        }
        else if(result == SX1268Error::RxTimeout) {
            return false;
        }
        else {
            Serial.print("Lora error on rx ");
            Serial.println((int)result);

            // Attempt to recover from communication failure.
            (void)init();

            return false;
        }
    }

private:
    /// SX1268 LoRa transceiver driver.
    SX1268 lora;
};