#pragma once

#include <RH_RF95.h>

#include "errors.h"
#include "hal.h"
#include "pins.h"
#include <TCAL9539.h>
#include <SX126x-Arduino.h>

extern uint8_t lora_payload[255];
extern bool tx_done;
extern bool rx_done;
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

//        Serial.println("Sending bytes");
        tx_done = false;
        Radio.Send((uint8_t*) &data, sizeof(T));

        for(int i = 1;; i++){
            if (tx_done) {
                break;
            } else if (i % 1024 == 0) {
                Serial.println("Slow tx!");
            }
            // Sometimes there is a desync between tx done and the actual transmit
            // So we just break so that we don't lose too much telemetry.
            if (i % 1024 * 2 == 0) {
                break;
            }
            delay(1);
        }

    }

    /**
     * @brief Reads message from the LoRa
     * 
     * @param write The buffer to write the data to
     * 
     * @return bool indicating a successful read and write to buffer
    */
    template<typename T>
    bool read(T* write, int wait_milliseconds) {
        static_assert(sizeof(T) <= 0xFF, "The data type to receive is too large");
        uint8_t len = sizeof(T);
        rx_done = false;
        // set receive mode
        Radio.Rx(wait_milliseconds);

        // busy wait for interrupt signalling
        for(int i = 1; i < wait_milliseconds; i++){
            THREAD_SLEEP(1);
            if(rx_done){
                break;
            }
        }
        memcpy(write, lora_payload, len);

        return false;
    }

private:
    hw_config hwConfig;
    bool led_state;
};
