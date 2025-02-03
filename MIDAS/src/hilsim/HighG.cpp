#include "sensors.h"
#include "SparkFun_Qwiic_KX13X.h"

QwiicKX134 KX;      // global static instance of the sensor

/**
 * @brief Initializes the high G sensor
 * 
 * @return Error Code
*/
ErrorCode HighGSensor::init() {
    KX.beginSPI(KX134_CS);
    if (!KX.initialize(DEFAULT_SETTINGS)) {
        return ErrorCode::HighGCouldNotBeInitialized;
    }

    if(!KX.setOutputDataRate(0xb)) {
        return ErrorCode::HighGCouldNotUpdateDataRate;
    }

    KX.setRange(3);
    return ErrorCode::NoError;
}

/**
 * @brief Reads and returns the data from the sensor
 * 
 * @return a HighGData packet with current acceleration in all three axes
*/
HighGData HighGSensor::read() {
    auto data = KX.getAccelData();
    return highg;
}
