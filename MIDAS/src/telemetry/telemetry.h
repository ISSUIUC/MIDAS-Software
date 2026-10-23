#pragma once

#include "telemetry_packet.h"
#include "flight-systems/rocket_state.h"
#include <util/errors.h>
#include "util/led.h"
#include "logging/esp_eeprom.h"

#if defined(SILSIM)
#include "silsim/emulated_telemetry.h"
#elif defined(HILSIM)
#include "hilsim/sensors/telemetry_backend.h"
#else
#include "hardware/telemetry_backend.h"
#endif

/**
 * @class Telemetry
 * * @brief wraps the telemetry system to create and send a packet
*/
class Telemetry {
public:
    /**
     * @brief Default constructor for Telemetry.
     */
    Telemetry() = default;
    
    /**
     * @brief Construct a new Telemetry object with a specific backend transceiver.
     * * @param backend The telemetry hardware or simulation backend to move into this instance.
     */
    explicit Telemetry(TelemetryBackend&& backend);

    /**
     * @brief Initializes the telemetry hardware backend.
     * * @return ErrorCode Returns ErrorCode::NoErrors on success, or a specific error code on failure.
     */
    ErrorCode __attribute__((warn_unused_result)) init();

    /**
     * @brief Composes a telemetry packet and transmits it over the backend transceiver.
     * * @param rocket_data Reference to the current state and sensor data of the rocket.
     * @param eeprom Reference to the EEPROM configuration and logging system.
     * @param led Reference to the LED controller to handle visual status signaling during transmission.
     */
    void transmit(RocketData& rocket_data, const MIDASEEPROM& eeprom, LEDController& led);
    
    /**
     * @brief Checks for and receives incoming telemetry commands within a specified timeout.
     * * @param command Pointer to the command structure where received data will be stored.
     * @param wait_milliseconds The maximum time to block waiting for a packet, in milliseconds.
     * @return true If a command was successfully received.
     * @return false If the timeout expired or an error occurred.
     */
    bool receive(TelemetryCommand* command, int wait_milliseconds);
    
    /**
     * @brief Sends an acknowledgment packet back to the ground station for a received command.
     */
    void acknowledgeReceived();
    
    /**
     * @brief Sets the transmission/reception frequency of the telemetry transceiver.
     * * @param frequency target frequency in MHz.
     * @return ErrorCode Returns ErrorCode::NoErrors on success, or an error code if the frequency is invalid/unsupported.
     */
    ErrorCode setFrequency(float frequency);
    
    /**
     * @brief Sets the FreeRTOS mutex used to synchronize access to the shared SPI bus.
     * * @param mtx The SemaphoreHandle_t representing the SPI mutex.
     */
    void set_spi_mutex(SemaphoreHandle_t mtx) { backend.set_spi_mutex(mtx); }
private:
    /**
     * @brief Total number of valid telemetry commands received during this session.
     */
    int received_count;
    
    /**
     * @brief Gathers flight data and EEPROM states into a standardized TelemetryPacket structure.
     * * @param data Reference to the source rocket flight and sensor telemetry data.
     * @param eeprom Reference to the system EEPROM configuration.
     * @return TelemetryPacket The formatted packet ready for transmission.
     */
    TelemetryPacket makePacket(RocketData& data, const MIDASEEPROM& eeprom);

    /**
     * @brief Underlying hardware, HIL, or SIL transceiver driver.
     */
    TelemetryBackend backend;
};