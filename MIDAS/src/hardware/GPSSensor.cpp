#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <SparkFun_u-blox_GNSS_v3.h>

#include "pins.h"
#include "sensors.h"
#include "flight-systems/sensor_data.h"

// see systems.cpp
extern SemaphoreHandle_t i2c_mutex;

/**
 * @class MIDASUbloxGNSS
 * @brief Wrapper around the SparkFun u-blox GNSS driver that provides
 *        thread-safe access to the shared I²C bus.
 *
 * The SparkFun library supports user-defined locking primitives. This
 * implementation uses the system-wide FreeRTOS I²C mutex to ensure that
 * multiple tasks cannot access the I²C bus simultaneously.
 */
class MIDASUbloxGNSS : public SFE_UBLOX_GNSS {
protected:
    /**
     * @brief Indicates that locking is handled externally.
     *
     * @return Always returns true.
     */
    bool createLock(void) override {
        return true;
    }

    /**
     * @brief Acquires exclusive access to the shared I²C bus.
     *
     * Blocks until the global I²C mutex becomes available.
     *
     * @return Always returns true.
     */
    bool lock(void) override {
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        return true;
    }

    /**
     * @brief Releases exclusive access to the shared I²C bus.
     */
    void unlock(void) override {
        xSemaphoreGive(i2c_mutex);
    }

    /**
     * @brief No-op since the mutex is owned by the system.
     */
    void deleteLock(void) override {
    }
};

/**
 * @brief Singleton instance of the u-blox GNSS interface.
 */
MIDASUbloxGNSS ublox;

/**
 * @brief Initializes the GPS receiver.
 *
 * Configures the receiver for airborne operation, enables both UBX and
 * NMEA output over I²C, sets a 10 Hz navigation update rate, and enables
 * automatic PVT (Position, Velocity, Time) messages.
 *
 * @return ErrorCode::NoError if initialization succeeds.
 * @return ErrorCode::GPSCouldNotBeInitialized if communication with the
 *         receiver fails.
 */
ErrorCode GPSSensor::init() {
    if (!ublox.begin()) {
        return ErrorCode::GPSCouldNotBeInitialized;
    }

    // Configure the receiver for high-dynamics flight applications.
    ublox.setDynamicModel(DYN_MODEL_AIRBORNE4g);

    // Enable both UBX and NMEA output over I²C.
    ublox.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);

    // Set the navigation solution update rate to 10 Hz.
    ublox.setMeasurementRate(100);

    // Automatically retrieve Position, Velocity, and Time data.
    ublox.setAutoPVT(true);

    return ErrorCode::NoError;
}


/**
 * @brief Reads the most recent GPS navigation solution.
 *
 * Retrieves the latest Position, Velocity, and Time (PVT) data from the
 * receiver. Altitude and ground speed are converted from millimeters to
 * meters before being returned.
 *
 * @return GPS data packet containing:
 *         - Latitude
 *         - Longitude
 *         - Altitude (m)
 *         - Ground speed (m/s)
 *         - GNSS fix type
 *         - Satellites in view
 *         - Unix timestamp
 */
GPS GPSSensor::read() {
    return GPS{
        ublox.getLatitude(),
        ublox.getLongitude(),
        (float) ublox.getAltitude() / 1000.f,
        (float) ublox.getGroundSpeed() / 1000.f,
        ublox.getFixType(),
        ublox.getSIV(),
        ublox.getUnixEpoch()
    };
}

/**
 * @brief Checks whether a valid PVT solution is available.
 *
 * @return True if fresh Position, Velocity, and Time data is available;
 *         false otherwise.
 */
bool GPSSensor::valid() {
    return ublox.getPVT();
}