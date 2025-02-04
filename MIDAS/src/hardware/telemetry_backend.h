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
    ErrorCode __attribute__((warn_unused_result)) init();

    int16_t getRecentRssi();
    void setFrequency(float frequency);

    /**
     * @brief This function transmits data from the struct provided as
     * the parameter (data collected from sensor suite) to the
     * ground station. The function also switches to a new commanded
     * frequency based on a previously received command and waits for
     * a response from the ground station.
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

        lora.send((uint8_t*) &data, sizeof(T));

        THREAD_SLEEP(100);

    }

    /**
     * @brief Reads message from the LoRa
     * 
     * @param write The buffer to write the data to
     * 
     * @return bool indicating a successful read and write to buffer
    */
    template<typename T>
    int read(T* write, int wait_milliseconds) {
        static_assert(sizeof(T) <= 0xFF, "The data type to receive is too large");
        uint8_t len = sizeof(T);
        // set receive mode
        return lora.recv((uint8_t*) write, len, wait_milliseconds);
    }

private:
    SX1268 lora;
    bool led_state;
};
