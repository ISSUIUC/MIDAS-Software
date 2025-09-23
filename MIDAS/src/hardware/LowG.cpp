#include "sensors.h"
#include "PL_ADXL355.h"

PL::ADXL355 sensor(ADXL355_CS);         //singleton object for the adxl

/**
 * @brief Initializes the low G sensor
 * 
 * @return Error Code
*/
ErrorCode LowGSensor::init() {
    ErrorCode error = ErrorCode::NoError;
    sensor.begin();
    sensor.setRange(PL::ADXL355_Range::range2g);
    sensor.setOutputDataRate(PL::ADXL355_OutputDataRate::odr1000);
    // todo set low pass filter frequency to 250hx
    sensor.enableMeasurement();
    return error;
}

/** 
 * @brief Reads and returns the data from the sensor
 * 
 * @return a_m_per_s LowGData packet with current acceleration in all three axes
*/
LowGData LowGSensor::read()
{
    auto data = sensor.getAccelerations();

    return { data.x, data.y, data.z };
}
