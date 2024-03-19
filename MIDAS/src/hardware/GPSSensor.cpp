#include "sensors.h"
#include "pins.h"
#include "MicroNMEA.h"
#include "teseo_liv3f_class.h"
#include "sensor_data.h"
#include <Wire.h>

GpioAddress gps_reset(2, 017);

// Replace "Wire" with a reference to the I2C object to be used by the sensor
TeseoLIV3F teseo(&Wire, gps_reset, GPS_ENABLE);

ErrorCode GPSSensor::init() {
    // Remember to change any RESET related operations to use the GPIO expander
    teseo.init();
//    if (!teseo.init()) {
//        return ErrorCode::GPSCouldNotBeInitialized;
//    }
    return ErrorCode::NoError;
}

GPS GPSSensor::read() {
    teseo.update();
    GPGGA_Info_t gpgga_message = teseo.getGPGGAData();
    GPRMC_Info_t gprmc_message = teseo.getGPRMCData();
    // Divide latitude and longitude by 100 because NMEA data is ddmmm.mmm or something like that
    // Also check if it's north or south
    float lat = gpgga_message.xyz.lat / 100.f * ((gpgga_message.xyz.ns == 'N') ? 1 : -1);
    float lon = gpgga_message.xyz.lon / 100.f * ((gpgga_message.xyz.ns == 'E') ? 1 : -1);
    float alt = gpgga_message.xyz.alt;
    float v = gprmc_message.speed;
    uint16_t sat_count = gpgga_message.sats;

    return GPS{lat, lon, alt, v, sat_count};
}
