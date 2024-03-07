#pragma once

#include <RH_RF95.h>

#include "errors.h"
#include "hal.h"


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
//        digitalWrite(led_pin, led_state);
//        led_state = !led_state;

        Serial.println("Sending bytes"); Serial.flush();
        rf95.send((uint8_t*) &data, sizeof(T));
        Serial.println("Sent bytes, waiting for send"); Serial.flush();
        rf95.waitPacketSent();
        Serial.println("Finishing sending, waiting"); Serial.flush();
        THREAD_SLEEP(170);
    }

    template<typename T>
    bool read(T* write) {
        static_assert(sizeof(T) <= RH_RF95_MAX_MESSAGE_LEN, "The data type to receive is too large");
        uint8_t len = sizeof(T);
        if (rf95.available() && rf95.recv((uint8_t*) write, &len)) {
            return len == sizeof(T);
        } else {
            return false;
        }
    }

private:
    RH_RF95 rf95;

//    uint8_t led_pin;
//    bool led_state;
};
