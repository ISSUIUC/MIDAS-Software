#include "sensors.h"
#include "pins.h"
#include "MicroNMEA.h"
#include "teseo_liv3f_class.h"
#include "sensor_data.h"
#include <Wire.h>

// Replace "Wire" with a reference to the I2C object to be used by the sensor
TeseoLIV3F teseo(&Wire, GPS_RESET, GPS_ENABLE);

ErrorCode GPSSensor::init() {
    teseo.init();
    return ErrorCode();
}

GPS GPSSensor::read() {
    teseo.update();
    auto gpgga_message = teseo.getGPGGAData();
    float lat = gpgga_message.xyz.lat;
    float lon = gpgga_message.xyz.lon;
    float alt = gpgga_message.xyz.alt;

    // Replace below statement with a valid velocity read from sensor. 
    // GPRMC is the NMEA message that has the relevant data (afaik)
    Velocity vel();
    int sat_count = gpgga_message.sats;

    return GPS(lat, lon, alt, vel, sat_count);
}