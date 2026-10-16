#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <SparkFun_u-blox_GNSS_v3.h>

#include "simulated_sensors.h"

// see systems.cpp
extern SemaphoreHandle_t i2c_mutex;

// override ublox for using the i2c mutex
class MIDASUbloxGNSS : public SFE_UBLOX_GNSS {
protected:
    bool createLock(void) override {
        return true;
    }
    bool lock(void) override {
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        return true;
    }
    void unlock(void) override {
        xSemaphoreGive(i2c_mutex);
    }
    void deleteLock(void) override {
    }
};

MIDASUbloxGNSS ublox;

/**
 * @brief Initializes GPS, returns NoError
 * 
 * @return Error code
 */
ErrorCode GPSSensor::init() {
    silsim_init("GPS");

    return ErrorCode::NoError;
}


/**
 * @brief Reads the GPS data from the sensor (lat, long, altitude, sat count, etc)
 * 
 * @return GPS data packet
 */
GPS GPSSensor::read() {
    std::map<double> data = silsim_read("GPS");
    // altitude and ground speed were divided by 1000 in HIL and I did not include it
    return GPS{data["latitude"], data, (float) ublox.getAltitude() / 1000.f, (float) ublox.getGroundSpeed() / 1000.f, data["fix_type"], data["sats_in_view"], data["time"]};
}

bool GPSSensor::valid() {
    return true;
}
