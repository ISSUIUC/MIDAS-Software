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
    GPGGA_Info_t gpgga_message = teseo.getGPGGAData();
    GPRMC_Info_t gprmc_message = teseo.getGPRMCData();
    float lat = gpgga_message.xyz.lat;
    float lon = gpgga_message.xyz.lon;
    float alt = gpgga_message.xyz.alt;
    float v = gprmc_message.speed;
    uint16_t sat_count = gpgga_message.sats;

    return GPS{lat, lon, alt, v, sat_count};
}