#pragma once

#include "errors.h"
#include "sensor_data.h"
#include "hardware/pins.h"


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
 * @struct LowGLSM interface
 */
// struct LowGLSMSensor {
//     ErrorCode init();
//     LowGLSM read();
// };


/**
 * @struct Voltage interface
 */
struct VoltageSensor {
    ErrorCode init();
    Voltage read();
};

// /**
//  * @struct BNO interface
//  */
// struct OrientationSensor {
//     Orientation initial_orientation;
//     Quaternion initial_quaternion;
//     uint8_t initial_flag;

//     float prev_x = 0;
//     float prev_y = 0;
//     float prev_z = 0;
//     float prev_tilt = 0;

//     ErrorCode init();
//     Orientation read();
// };

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
