/**
 * @file GPSSensor.cpp
 * 
 * @brief Holds the function definitions for the Teseo GPS
*/

#include "sensors.h"
#include "pins.h"
#include "MicroNMEA.h"
#include "teseo_liv3f_class.h"
#include "sensor_data.h"
#include <Wire.h>

TeseoLIV3F teseo(&Wire, GPS_RESET, GPS_ENABLE);     //singleton for the teseo gps

/**
 * @brief Initializes GPS, returns NoError
 * 
 * @return Error code
*/
ErrorCode GPSSensor::init() {
    teseo.init();       //function always returns ok for some reason

    return ErrorCode::NoError;
}

/**
 * @brief Reads the GPS data from the sensor (lat, long, altitude, sat count, etc)
 * 
 * @return GPS data packet
*/
GPS GPSSensor::read() {
    teseo.update();
    GPGGA_Info_t gpgga_message = teseo.getGPGGAData();
    GPRMC_Info_t gprmc_message = teseo.getGPRMCData();
    // Also check if it's north or south
    float64_t lat = gpgga_message.xyz.lat;
    float64_t lon = gpgga_message.xyz.lon;

    //                                            d ddm m.mm mmm
    // the max value of an unsigned 32 bit int is 2,147,4 83,647
    // Since the maximum longitude is 180, we can store 3 degree digits, and
    // 7 minute digits, which is all we need, because NMEA gives 6 minute digits.
    // See https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual-Rev2.1-Dec07.pdf
    int32_t lat_int = static_cast<int32_t>(lat*100000);
    int32_t lon_int = static_cast<int32_t>(lon*100000);

    lat_int *= (gpgga_message.xyz.ns == 'N') ? 1 : -1;
    lon_int *= (gpgga_message.xyz.ew == 'E') ? 1 : -1;
    float alt = gpgga_message.xyz.alt;
    float v = gprmc_message.speed;
    uint16_t sat_count = gpgga_message.sats;

    return GPS{lat_int, lon_int, alt, v, sat_count};
}
