#include <Wire.h>

#include <SparkFun_u-blox_GNSS_v3.h>

#include "pins.h"
#include "sensors.h"
#include "sensor_data.h"

SFE_UBLOX_GNSS ublox;

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
	ublox.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
    // Set the measurment rate faster than one HZ if necessary
    // ublox.setMeasurementRate(100);
	ublox.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

    ublox.logRXMSFRBX();
	//This will pipe all NMEA sentences to the serial port so we can see them
    return ErrorCode::NoError;
}


/**
 * @brief Reads the GPS data from the sensor (lat, long, altitude, sat count, etc)
 * 
 * @return GPS data packet
 */
GPS GPSSensor::read() {
    ublox.extractFileBufferData(data, 512);
    for (int i = 0; i < 512; i++) {
        Serial.print(data[i]);
        Serial.print(" ");
    }
    Serial.println();
    ublox.checkUblox();
    return GPS{ublox.getLatitude(), ublox.getLongitude(), (float) ublox.getAltitude() / 1000.f, (float) ublox.getGroundSpeed() / 1000.f, ublox.getFixType(), ublox.getUnixEpoch()};
}

bool GPSSensor::valid() {
    return ublox.getPVT();
}
