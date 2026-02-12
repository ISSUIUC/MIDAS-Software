#include <SparkFun_MMC5983MA_Arduino_Library.h>

#include "sensors.h"
#include "hal.h"

SFE_MMC5983MA MMC5983;           // global static instance of the sensor

ErrorCode MagnetometerSensor::init() {
    if (MMC5983.begin(MAG_CS) == false) {                     // Checks if sensor is connected
        return ErrorCode::MagnetometerCouldNotBeInitialized;
    }
    return ErrorCode::NoError;
}

Magnetometer MagnetometerSensor::read() {
    // read from aforementioned global instance of sensor
    uint32_t cx, cy, cz;
    double X, Y, Z;

    MMC5983.getMeasurementXYZ(&cx, &cy, &cz);

    double sf = (double)(1 << 17);
    X = ((double)cx - sf)/sf;
    Y = ((double)cy - sf)/sf;
    Z = ((double)cz - sf)/sf;
    
    Magnetometer reading{X, Y, Z};
    return reading;
}