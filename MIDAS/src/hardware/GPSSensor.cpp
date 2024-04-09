#include "sensors.h"
#include "pins.h"
#include "MicroNMEA.h"
#include "teseo_liv3f_class.h"
#include "sensor_data.h"
#include <Wire.h>

// Replace "Wire" with a reference to the I2C object to be used by the sensor
TeseoLIV3F teseo(&Wire, GPS_RESET, GPS_ENABLE);

ErrorCode GPSSensor::init() {
    // Remember to change any RESET related operations to use the GPIO expander
    teseo.init();
//    if (!teseo.init()) {
//        return ErrorCode::GPSCouldNotBeInitialized;
//    }
    return ErrorCode::NoError;
}

// This is needed because GPS doesn't provide unix time and just gives
// dd mm yy
uint16_t months[12] = {
    0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
};

inline bool is_leapyear(int year) {
    return (!(year % 100 == 0) || (year % 400 == 0)) && year % 4 == 0;
}

GPS GPSSensor::read() {
    teseo.update();
    GPGGA_Info_t gpgga_message = teseo.getGPGGAData();
    GPRMC_Info_t gprmc_message = teseo.getGPRMCData();

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

    if (!is_leapyear(gprmc_message.date % 100)) {
        is_leap = true;
    }
    if (!is_leap && is_leapyear(gprmc_message.date % 100)) {
        is_leap = true;
        for (int i = 2; i < 12; i++) {
            months[i]++;
        }
    }

    uint32_t day = gprmc_message.date / 10000 * 86400;
    uint32_t month = gprmc_message.date / 100 % 100;
    uint32_t time = day + months[month - 1] * 86400 + gprmc_message.date % 100 * 31536000;
    // Sum everything together now
    uint32_t time_of_day = gprmc_message.utc.hh * 3600 + gprmc_message.utc.mm * 60 + gprmc_message.utc.ss;
    time += time_of_day;
    return GPS{lat_int, lon_int, alt, v, sat_count, time};
}
