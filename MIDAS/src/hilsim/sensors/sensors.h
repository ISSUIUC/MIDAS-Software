#pragma once

#include "errors.h"
#include "sensor_data.h"

/**
 * @struct IMUSensor
 */
struct IMUSensor {
    ErrorCode init();
    IMU read(); 
    IMU_SFLP read_sflp();
};

/**
 * @struct Magnetometer interface
 */
struct MagnetometerSensor {
    ErrorCode init();
    Magnetometer read();
};

/**
 * @struct Barometer interface
 */
struct BarometerSensor {
    ErrorCode init();
    Barometer read();
};


/**
 * @struct Voltage interface
 */
struct VoltageSensor {
    ErrorCode init();
    Voltage read();
};


/**
 * @struct GPS interface
 */
struct GPSSensor {
    ErrorCode init();
    bool valid();
    GPS read();
    bool is_leap = false;
};
/**
 * @struct Pyro interface
 */
struct Pyro {
    ErrorCode init();

    PyroState tick(FSMState fsm_state, AngularKalmanData angularkalman);
};
