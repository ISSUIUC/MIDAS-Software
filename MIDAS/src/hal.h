#pragma once

// the only thing #ifdef we need, theoretically
#ifdef SILSIM
#include "silsim/emulation.h"
#else
#include "FreeRTOSConfig.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#endif

#include "sensor_data.h"
#include "errors.h"

/**
 * @brief Delays the running thread.
 * @param millis The time to delay in milliseconds.
*/
#define THREAD_SLEEP(millis) vTaskDelay(pdMS_TO_TICKS(millis))

/**
 * @struct Sensors
 *
 * @brief holds all interfaces for all sensors on MIDAS
 */
template<typename HwImpl>
struct HwInterface {
    ErrorCode init_all() { return static_cast<HwImpl*>(this)->init_all(); };

    LowGData read_low_g() { return static_cast<HwImpl*>(this)->read_low_g(); }
    LowGLSMData read_low_g_lsm() { return static_cast<HwImpl*>(this)->read_low_g_lsm(); }
    HighGData read_high_g() { return static_cast<HwImpl*>(this)->read_high_g(); }
    BarometerData read_barometer() { return static_cast<HwImpl*>(this)->read_barometer(); }
    ContinuityData read_continuity() { return static_cast<HwImpl*>(this)->read_continuity(); }
    VoltageData read_voltage() { return static_cast<HwImpl*>(this)->read_voltage(); }
    bool is_orientation_ready() { return static_cast<HwImpl*>(this)->is_orientation_ready(); }
    OrientationData read_orientation() { return static_cast<HwImpl*>(this)->read_orientation(); }
    MagnetometerData read_magnetometer() { return static_cast<HwImpl*>(this)->read_magnetometer(); }
    bool is_gps_ready() { return static_cast<HwImpl*>(this)->is_gps_ready(); }
    GPSData read_gps() { return static_cast<HwImpl*>(this)->read_gps(); }

    void set_global_arm(bool to_high) { return static_cast<HwImpl*>(this)->set_global_arm(to_high); }
    void set_pin_firing(int which, bool to_high)  { return static_cast<HwImpl*>(this)->set_pin_firing(which, to_high); }

protected:
    HwInterface() = default;
};
