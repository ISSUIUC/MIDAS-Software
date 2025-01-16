#pragma once


#include "hardware_interface.h"
#include "rocket_state.h"
#include "buzzer.h"
#include "led.h"
#include "telemetry.h"
#include "pyro_controller.h"

#if defined(IS_SUSTAINER) && defined(IS_BOOSTER)
#error "Only one of IS_SUSTAINER and IS_BOOSTER may be defined at the same time."
#elif !defined(IS_SUSTAINER) && !defined(IS_BOOSTER)
#error "At least one of IS_SUSTAINER and IS_BOOSTER must be defined."
#endif

/**
 * @struct RocketData
 *
 * @brief holds all information about the rocket, sensors, and controllers
 */
struct RocketSystems {
    Sensors sensors;
    RocketData rocket_data;
    BuzzerController buzzer;
    LEDController led;
    Telemetry tlm;
    PyroController pyro;

    explicit RocketSystems(Sensors hardware);
    [[noreturn]] void begin();

private:
    ErrorCode init_systems();
};
