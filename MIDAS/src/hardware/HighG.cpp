#include "sensors.h"

// #include sensor library
#include "SparkFun_Qwiic_KX13X.h"

// global static instance of the sensor
QwiicKX134 KX;

/**
 * Initializes the high G data sensor, returns ErrorCode::CANNOT_INIT_KX134_CS if cannot initialize
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
 * Reads and returns the data from the sensor
 * @return a HighGData packet with current acceleration in all three axies
*/
HighGData HighGSensor::read() {
    auto data = KX.getAccelData();
    return HighGData(data.xData, data.yData, data.zData);
}
