#include <Adafruit_LIS3MDL.h>
#include <queue> 
#include "sensors.h"
#include "hal.h"

//extern std::queue<char[32]> logQueue; // global variable queue for logging the important status of the sensor.

Adafruit_LIS3MDL LIS3MDL;           // global static instance of the sensor

ErrorCode MagnetometerSensor::init() {
    if (!LIS3MDL.begin_SPI(LIS3MDL_CS)) {                     // Checks if sensor is connected
        return ErrorCode::MagnetometerCouldNotBeInitialized;
    }
    LIS3MDL.setOperationMode(LIS3MDL_CONTINUOUSMODE);         // Reading continuously, instead of single-shot or off
    LIS3MDL.setDataRate(LIS3MDL_DATARATE_155_HZ);
    LIS3MDL.setRange(LIS3MDL_RANGE_4_GAUSS);  
                    // Earth's magnetic field is 1/2 gauss, can detect high current
    //char message[32];
    //snprintf(message, sizeof(message), "Magnetometer init w/ mode:%d rate:%d range:%d", LIS3MDL_CONTINUOUSMODE, static_cast<int>(LIS3MDL_DATARATE_155_HZ), LIS3MDL_RANGE_4_GAUSS);
    //logQueue.push(message);
    return ErrorCode::NoError;
}//process profiling

Magnetometer MagnetometerSensor::read() {
    // read from aforementioned global instance of sensor
    LIS3MDL.read();

    float mx = LIS3MDL.x_gauss;
    float my = LIS3MDL.y_gauss;
    float mz = LIS3MDL.z_gauss;
    Magnetometer reading{mx, my, mz};
    return reading;
}
