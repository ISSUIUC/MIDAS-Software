#pragma once

#include "errors.h"
#include "sensor_data.h"
#include "hardware/pins.h"

/**
 * @struct Voltage interface
 */
struct VoltageSensor {
    ErrorCode init();
    VoltageSense read();
};