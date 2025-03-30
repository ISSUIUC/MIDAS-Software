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

enum class LED {
    BLUE = 0,
    RED = 1,
    ORANGE = 2,
    GREEN = 3
};

enum class Channel {
    A = 0,
    B = 1,
    C = 2,
    D = 3
};

enum class Camera {
    Side = 0,
    Bulkhead = 1
};

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

    void set_led(LED which, bool value) { return static_cast<HwImpl*>(this)->set_led(which, value); }

    void transmit_bytes(uint8_t* memory, size_t count) { return static_cast<HwImpl*>(this)->transmit_bytes(memory, count); }
    bool receive_bytes(uint8_t* memory, size_t count, int wait_milliseconds) { return static_cast<HwImpl*>(this)->receive_bytes(memory, count, wait_milliseconds); }

    template<typename T>
    void transmit(T* value) { return transmit_bytes((uint8_t*) value, sizeof(T)); }
    template<typename T>
    bool receive(T* value, int wait_milliseconds) { return receive_bytes((uint8_t*) value, sizeof(T), wait_milliseconds); }

    void set_global_arm(bool to_high) { return static_cast<HwImpl*>(this)->set_global_arm(to_high); }
    void set_pin_firing(Channel which, bool to_high)  { return static_cast<HwImpl*>(this)->set_pin_firing(which, to_high); }

    void set_camera_on(Camera which, bool on) { return static_cast<HwImpl*>(this)->set_camera_on(which, on); }
    void set_camera_source(Camera which) { return static_cast<HwImpl*>(this)->set_camera_source(which); }
    void set_video_transmit(bool on) { return static_cast<HwImpl*>(this)->set_video_transmit(on); }
    uint8_t get_camera_state() { return static_cast<HwImpl*>(this)->get_camera_state(); }

protected:
    HwInterface() = default;
};
