#include <SparkFun_MMC5983MA_Arduino_Library.h>

#include "sensors.h"
#include "hal.h"

SFE_MMC5983MA MMC5983;           // global static instance of the sensor

ErrorCode MagnetometerSensor::init() {
    // Checks if sensor is connected
    if (!MMC5983.begin(MMC5983_CS)) {
        return ErrorCode::MagnetometerCouldNotBeInitialized;
    }
    return ErrorCode::NoError;
}

Magnetometer MagnetometerSensor::read() {
    // read from aforementioned global instance of sensor
    uint32_t cx, cy, cz;
    double X, Y, Z;
    
    MMC5983.getMeasurementXYZ(&cx, &cy, &cz);
    
    // The magnetic field values are 18-bit unsigned. The _approximate_ zero (mid) point is 2^17
    // Here we scale each field to +/- 1.0 to make it easier to convert to Gauss
    // https://github.com/sparkfun/SparkFun_MMC5983MA_Magnetometer_Arduino_Library/tree/main/examples
    double sf = (double)(1 << 17);
    X = ((double)cx - sf)/sf;
    Y = ((double)cy - sf)/sf;
    Z = ((double)cz - sf)/sf;
    
    Magnetometer reading{Y, -X, -Z};
    return reading;
}