#include "sensors.h"
#include "PL_ADXL355.h"
#include "pins.h"

PL::ADXL355 sensor(ADXL355_CS);

ErrorCode LowGSensor::init()
{
    ErrorCode error = ErrorCode::NoError;
    sensor.begin();
    sensor.setRange(PL::ADXL355_Range::range2g);
    sensor.setOutputDataRate(PL::ADXL355_OutputDataRate::odr1000);
    // todo set low pass filter frequency to 250hx
    sensor.enableMeasurement();
    return error;
}

LowGData LowGSensor::read()
{
    auto data = sensor.getAccelerations();

    return { data.x, data.y, data.z };
}
