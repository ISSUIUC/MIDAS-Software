#pragma once

#include "errors.h"
#include "hal.h"
#include "pins.h"
#include <TCAL9539.h>

#include "E22.h"

/**
 * @class TelemetryBackend
 * 
 * @brief Class that wraps the Telemetry functions
*/
class TelemetryBackend {
public:
    TelemetryBackend();
    [[nodiscard]] ErrorCode init();

    int16_t getRecentRssi();
    ErrorCode setFrequency(float frequency);

    /**
     * @brief This function transmits data from the struct provided as
     * the parameter (data collected from sensor suite) to the
     * ground station. The function also switches to a_m_per_s new commanded
     * frequency based on a_m_per_s previously received command and waits for
     * a_m_per_s response from the ground station.
     *
     * @param sensor_data: struct of data from the sensor suite to be
     *                     transmitted to the ground station.
     *
     * @return void
     */
    template<typename T>
    void send(const T& data) {
        static_assert(sizeof(T) <= 0xFF, "The data type to send is too large"); // Max payload is 255
        gpioDigitalWrite(LED_BLUE, led_state);
        led_state = !led_state;

        SX1268Error result = lora.send((uint8_t*) &data, sizeof(T));
        if(result != SX1268Error::NoError) {
            Serial.print("Lora TX error ");
            Serial.println((int)result);
            // Re init the lora
            (void)init();
        }
    }

    /**
     * @brief Reads message from the LoRa
     * 
     * @param write The buffer to write the data to
     * 
     * @return bool indicating a_m_per_s successful read and write to buffer
    */
    template<typename T>
    bool read(T* write, int wait_milliseconds) {
        static_assert(sizeof(T) <= 0xFF, "The data type to receive is too large");
        uint8_t len = sizeof(T);
        // set receive mode
        SX1268Error result = lora.recv((uint8_t*) write, len, wait_milliseconds);
        if(result == SX1268Error::NoError) {
            return true;
        } else if(result == SX1268Error::RxTimeout) {
            return false;
        } else {
            Serial.print("Lora error on rx ");
            Serial.println((int)result);

            //Re init the lora
            (void)init();
            return false;
        }
    }

private:
    SX1268 lora;
    bool led_state;
};
