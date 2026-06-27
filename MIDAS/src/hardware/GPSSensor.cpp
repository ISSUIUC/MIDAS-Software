#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <SparkFun_u-blox_GNSS_v3.h>

#include "pins.h"
#include "sensors.h"
#include "flight-systems/sensor_data.h"

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
    if (!ublox.begin()) {
        return ErrorCode::GPSCouldNotBeInitialized;
    }

    ublox.setDynamicModel(DYN_MODEL_AIRBORNE4g);
    ublox.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);
    ublox.setMeasurementRate(100);              // 10 Hz full nav
    ublox.setAutoPVT(true);

    return ErrorCode::NoError;
}


/**
 * @brief Reads the GPS data from the sensor (lat, long, altitude, sat count, etc)
 * 
 * @return GPS data packet
 */
GPS GPSSensor::read() {
    return GPS{ublox.getLatitude(), ublox.getLongitude(), (float) ublox.getAltitude() / 1000.f, (float) ublox.getGroundSpeed() / 1000.f, ublox.getFixType(), ublox.getSIV(), ublox.getUnixEpoch()};
}

bool GPSSensor::valid() {
    return ublox.getPVT();
}
