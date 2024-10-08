#pragma once

#include <RH_RF95.h>

#include "errors.h"
#include "hal.h"
#include "TCAL9539.h"
#include "pins.h"

/**
 * @class TelemetryBackend
 * 
 * @brief Class that wraps the Telemetry functions
*/
class TelemetryBackend {
public:
    TelemetryBackend();
    ErrorCode __attribute__((warn_unused_result)) init();

    int8_t getRecentRssi();
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
        static_assert(sizeof(T) <= RH_RF95_MAX_MESSAGE_LEN, "The data type to send is too large");
        // gpioDigitalWrite(LED_BLUE, led_state);
        led_state = !led_state;

//        Serial.println("Sending bytes");
        rf95.send((uint8_t*) &data, sizeof(T));
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

    /**
     * @brief Reads message from the LoRa
     * 
     * @param write The buffer to write the data to
     * 
     * @return bool indicating a successful read and write to buffer
    */
    template<typename T>
    bool read(T* write, int wait_milliseconds) {
        static_assert(sizeof(T) <= RH_RF95_MAX_MESSAGE_LEN, "The data type to receive is too large");
        uint8_t len = sizeof(T);

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

        if (rf95.available() && rf95.recv((uint8_t*) write, &len)) {
            if (sizeof(T) == len) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }

private:
    RH_RF95 rf95;

    bool led_state;
};
