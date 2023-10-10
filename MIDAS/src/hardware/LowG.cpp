#include "sensors.h"
#include "lib/Adxl355.h"
#include "pins.h"

// #include sensor library

// global static instance of the sensor

Adxl355 sensor;

void LowGSensor::calibrate()
{
    sensor.calibrateSensor(ADXL355_CS);
}

ErrorCode LowGSensor::init()
{
    ErrorCode error = ErrorCode::NoError;
    sensor.initSPI(SPI);
    sensor.start();
    delay(1000);

    if (sensor.isDeviceRecognized())
    {
        // Defaults to 2G range and Output Data Rate: 4000Hz and Low Pass Filter: 1000Hz
        sensor.initializeSensor();

        // Swap check if the sensor is changed from different values
        if (Adxl355::RANGE_VALUES::RANGE_2G != sensor.getRange())
        {
            error = ErrorCode::LowGRangeCouldNotBeSet;
        }

        if (Adxl355::ODR_LPF::ODR_4000_AND_1000 != sensor.getOdrLpf())
        {
            error = ErrorCode::LowGODRLPFCouldNotBeSet;
        }
    }
    else
    {
        error = ErrorCode::LowGCouldNotBeInitialized;
    }

    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    return error;
}

LowGData LowGSensor::read()
{
    // read from aforementioned global instance of sensor
    auto data = sensor.getAccel();

    return LowGData{data.x, data.y, data.z};
}