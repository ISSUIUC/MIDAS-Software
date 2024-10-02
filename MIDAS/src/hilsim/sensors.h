#pragma once

#include "errors.h"
#include "sensor_data.h"
#include "hardware/pins.h"


struct GpioAddress {
    GpioAddress(int gpio_id, int pin_id) : gpio_id(gpio_id), pin_offset(pin_id) {}
    uint8_t gpio_id; //id of the expander
    uint8_t gpio_address; //i2c address of expander
    uint8_t port_idx; //0 = bottom 8 bits, 1 = top 8 bits
    uint8_t pin_offset; //[0,7] bit in port, 8 * port_idx + pin_offset = pin idx;
    bool is_valid; //whether the address is valid
};

/**
 * @struct LowG interface
 */
struct LowGSensor {
    ErrorCode init();
    LowGData read();
};

/**
 * @struct HighG interface
 */
struct HighGSensor {
    ErrorCode init();
    HighGData read();
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
struct LowGLSMSensor {
    ErrorCode init();
    LowGLSM read();
};

/**
 * @struct Continuity interface
 */
struct ContinuitySensor {
    ErrorCode init();
    Continuity read();
};

/**
 * @struct Voltage interface
 */
struct VoltageSensor {
    ErrorCode init();
    Voltage read();
};

/**
 * @struct BNO interface
 */
struct OrientationSensor {
    Orientation initial_orientation;
    uint8_t initial_flag;
    ErrorCode init();
    Orientation read();
};

/**
 * @struct GPS interface
 */
struct GPSSensor {
    ErrorCode init();
    GPS read();
    bool is_leap = false;
};

/**
 * @struct Pyro interface
 */
struct Pyro {
    ErrorCode init();
    PyroState tick(FSMState fsm_state, Orientation orientation);
};
