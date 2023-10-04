#include "sensors.h"

// #include sensor library
#include "SparkFun_Qwiic_KX13X.h"

// global static instance of the sensor
QwiicKX134 KX;

ErrorCode HighGSensor::init() {
    KX.beginSPI(KX134_CS);
    if (!KX.initialize(DEFAULT_SETTINGS)) {
        return ErrorCode::CANNOT_INIT_KX134_CS;
    }

    KX.setRange(3);
    return ErrorCode::NoError;
}


HighGData HighGSensor::read() {
    // read from aforementioned global instance of sensor
    auto data = KX.getAccelData();
    ax = data.xData;
    ay = data.yData;
    az = data.zData;

    return HighGData{ax, ay, az};
}