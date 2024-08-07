#include "systems.h"

#include "hal.h"
#include "gnc/yessir.h"

#if defined(IS_SUSTAINER) && defined(IS_BOOSTER)
#error "Only one of IS_SUSTAINER and IS_BOOSTER may be defined at the same time."
#elif !defined(IS_SUSTAINER) && !defined(IS_BOOSTER)
#error "At least one of IS_SUSTAINER and IS_BOOSTER must be defined."
#endif


/**
 * @brief These are all the functions that will run in each task
 * Each function has a `while (true)` loop within that should not be returned out of or yielded in any way
 * 
 * @param name the name of the thread, replace with the actual name
 * @param arg the config file for the rocket
 */


/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(logger, RocketSystems* arg) {
    log_begin(arg->log_sink);
    while (true) {
        log_data(arg->log_sink, arg->rocket_data);

        arg->rocket_data.log_latency.tick();

        THREAD_SLEEP(1);
    }
}

/**
 * See \ref barometer_thread
 */
DECLARE_THREAD(barometer, RocketSystems* arg) {
    while (true) {
        Barometer reading = arg->sensors.barometer.read();
        arg->rocket_data.barometer.update(reading);
        THREAD_SLEEP(6);
    }
}

/**
 * See \ref accelerometers_thread
 */
DECLARE_THREAD(accelerometers, RocketSystems* arg) {
    while (true) {
#ifdef IS_SUSTAINER
        LowGData lowg = arg->sensors.low_g.read();
        arg->rocket_data.low_g.update(lowg);
#endif
        LowGLSM lowglsm = arg->sensors.low_g_lsm.read();
        arg->rocket_data.low_g_lsm.update(lowglsm);
        HighGData highg = arg->sensors.high_g.read();
        arg->rocket_data.high_g.update(highg);
        THREAD_SLEEP(2);
    }
}

/**
 * See \ref orientation_thread
 */
DECLARE_THREAD(orientation, RocketSystems* arg) {
    while (true) {
        Orientation reading = arg->sensors.orientation.read();
        if (reading.has_data) {
            arg->rocket_data.orientation.update(reading);
        }

        THREAD_SLEEP(100);
    }
}

/**
 * See \ref magnetometer_thread
 */
DECLARE_THREAD(magnetometer, RocketSystems* arg) {
    while (true) {
        Magnetometer reading = arg->sensors.magnetometer.read();
        arg->rocket_data.magnetometer.update(reading);
        THREAD_SLEEP(50);  //data rate is 155hz so 7 is closest
    }
}

/**
 * See \ref i2c_thread
 * 
 * @note
 * this thread holds all instructios for all components on i2c
 */
DECLARE_THREAD(i2c, RocketSystems* arg) {
    int i = 0;

    while (true) {
        if (i % 10 == 0) {
            GPS reading = arg->sensors.gps.read();
            arg->rocket_data.gps.update(reading);

            FSMState current_state = arg->rocket_data.fsm_state.getRecentUnsync();
            PyroState new_pyro_state = arg->sensors.pyro.tick(current_state, arg->rocket_data.orientation.getRecentUnsync());
            arg->rocket_data.pyro.update(new_pyro_state);

            Continuity reading2 = arg->sensors.continuity.read();
            arg->rocket_data.continuity.update(reading2);

            Voltage reading3 = arg->sensors.voltage.read();
            arg->rocket_data.voltage.update(reading3);
        }

        arg->led.update();
        i += 1;

        THREAD_SLEEP(10);
    }
}

/**
 * See \ref fsm_thread
 */
DECLARE_THREAD(fsm, RocketSystems* arg) {
    FSM fsm {};
    bool already_played_freebird = false;
    while (true) {
        FSMState current_state = arg->rocket_data.fsm_state.getRecentUnsync();
        StateEstimate state_estimate(arg->rocket_data);

        FSMState next_state = fsm.tick_fsm(current_state, state_estimate);

        arg->rocket_data.fsm_state.update(next_state);

        if (current_state == FSMState::STATE_SUSTAINER_IGNITION && !already_played_freebird) {
            arg->buzzer.play_tune(free_bird, FREE_BIRD_LENGTH);
            already_played_freebird = true;
        }

        THREAD_SLEEP(50);
    }
}

/**
 * See \ref buzzer_thread
 */
DECLARE_THREAD(buzzer, RocketSystems* arg) {
    while (true) {
        arg->buzzer.tick();

        THREAD_SLEEP(10);
    }
}

/**
 * See \ref kalman_thread
 */
DECLARE_THREAD(kalman, RocketSystems* arg) {
    yessir.initialize();
    TickType_t last = xTaskGetTickCount();

    while (true) {
        // add the tick update function
        Barometer current_barom_buf = arg->rocket_data.barometer.getRecentUnsync();
        LowGData current_accelerometer = arg->rocket_data.low_g.getRecentUnsync();
        Acceleration current_accelerations = {
            .ax = current_accelerometer.ax,
            .ay = current_accelerometer.ay,
            .az = current_accelerometer.az
        };
        float dt = pdTICKS_TO_MS(xTaskGetTickCount() - last) / 1000.0f;

        yessir.tick(dt, 13.0, current_barom_buf, current_accelerations);
        KalmanData current_state = yessir.getState();

        arg->rocket_data.kalman.update(current_state);

        last = xTaskGetTickCount();

        THREAD_SLEEP(50);
    }
}

/**
 * See \ref telemetry_thread
 */
DECLARE_THREAD(telemetry, RocketSystems* arg) {
    while (true) {
        arg->tlm.transmit(arg->rocket_data, arg->led);

        THREAD_SLEEP(1);
    }
}

/**
 * @brief initializes all systems
*/
#define INIT_SYSTEM(s) do { ErrorCode code = (s).init(); if (code != NoError) { return code; } } while (0)
ErrorCode init_systems(RocketSystems& systems) {
    gpioDigitalWrite(LED_ORANGE, HIGH);
#ifdef IS_SUSTAINER
    INIT_SYSTEM(systems.sensors.low_g);
    INIT_SYSTEM(systems.sensors.orientation);
#endif
    INIT_SYSTEM(systems.log_sink);
    INIT_SYSTEM(systems.sensors.high_g);
    INIT_SYSTEM(systems.sensors.low_g_lsm);
    INIT_SYSTEM(systems.sensors.barometer);
    INIT_SYSTEM(systems.sensors.magnetometer);
    INIT_SYSTEM(systems.sensors.continuity);
    INIT_SYSTEM(systems.sensors.voltage);
    INIT_SYSTEM(systems.sensors.pyro);
    INIT_SYSTEM(systems.led);
    INIT_SYSTEM(systems.buzzer);
    INIT_SYSTEM(systems.tlm);
    INIT_SYSTEM(systems.sensors.gps);
    gpioDigitalWrite(LED_ORANGE, LOW);
    return NoError;
}
#undef INIT_SYSTEM

/**
 * @brief
 * Creates all threads for each sensor, FSM, Kalman algorithm, and data logging member
 * Starts thread scheduler to actually start doing jobs
*/
[[noreturn]] void begin_systems(RocketSystems* config) {
    Serial.println("Starting Systems...");
    ErrorCode init_error_code = init_systems(*config);
    if (init_error_code != NoError) {
        // todo some message probably
        while (true) {
            Serial.print("Had Error: ");
            Serial.print((int) init_error_code);
            Serial.print("\n");
            Serial.flush();
            update_error_LED(init_error_code);
        }
    }

#ifdef IS_SUSTAINER
    START_THREAD(orientation, SENSOR_CORE, config, 10);
#endif

    START_THREAD(logger, DATA_CORE, config, 15);
    START_THREAD(accelerometers, SENSOR_CORE, config, 13);
    START_THREAD(barometer, SENSOR_CORE, config, 12);
    START_THREAD(i2c, SENSOR_CORE, config, 9);
    START_THREAD(magnetometer, SENSOR_CORE, config, 11);
    START_THREAD(kalman, SENSOR_CORE, config, 7);
    START_THREAD(fsm, SENSOR_CORE, config, 8);
    START_THREAD(buzzer, SENSOR_CORE, config, 6);
    START_THREAD(telemetry, SENSOR_CORE, config, 15);

    config->buzzer.play_tune(free_bird, FREE_BIRD_LENGTH);
    
    while (true) {
        THREAD_SLEEP(1000);
        Serial.print("Running (Log Latency: ");
        Serial.print(config->rocket_data.log_latency.getLatency());
        Serial.println(")");
    }
}

//void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char* pcTaskName){
//    Serial.println("OVERFLOW");
//    Serial.println((char*)pcTaskName);
//}
