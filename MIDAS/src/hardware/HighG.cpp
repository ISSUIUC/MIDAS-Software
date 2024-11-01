#include "sensors.h"
#include "SparkFun_Qwiic_KX13X.h"
#include <queue>

QwiicKX134 KX;      // global static instance of the sensor
extern std::queue<std::string> logQueue; 

/**
 * @brief Initializes the high G sensor
 * 
 * @return Error Code
*/
ErrorCode HighGSensor::init() {
    KX.beginSPI(KX134_CS);
    if (!KX.initialize(DEFAULT_SETTINGS)) {
        
        logQueue.push("HighGCouldNotBeInitialized");//process profiling
        return ErrorCode::HighGCouldNotBeInitialized;
    }

    if(!KX.setOutputDataRate(0xb)) {
        logQueue.push("HighGCouldNotUpdateDataRate");//process profiling
        return ErrorCode::HighGCouldNotUpdateDataRate;
    }

    
    logQueue.push("HighGInitialized");//process profiling
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
    return HighGData(data.xData, data.yData, data.zData);
}
