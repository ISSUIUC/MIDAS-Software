#include "systems.h"

#include "hal.h"
#include "sensors.h"
#include "data_logging.h"
#include "buzzer.h"

#include "gnc/yessir.h"

#if defined(IS_SUSTAINER) && defined(IS_BOOSTER)
#error "Only one of IS_SUSTAINER and IS_BOOSTER may be defined at the same time."
#elif !defined(IS_SUSTAINER) && !defined(IS_BOOSTER)
#error "At least one of IS_SUSTAINER and IS_BOOSTER must be defined."
#endif

/**
 * These are all the functions that will run in each task
 * Each function has a `while (true)` loop within that should not be returned out of or yielded in any way
 * 
 * @param name the name of the thread, replace with the actual name
 * @param arg the config file for the rocket
 */
DECLARE_THREAD(logger, RocketSystems* arg) {
    log_begin(arg->log_sink);
    while (true) {
        log_data(arg->log_sink, arg->rocket_data);

        arg->rocket_data.log_latency.tick();

        THREAD_SLEEP(30);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(barometer, RocketSystems* arg) {
    while (true) {
        Barometer reading = arg->sensors.barometer.read();
        arg->rocket_data.barometer.update(reading);
        THREAD_SLEEP(50);
    }
}


DECLARE_THREAD(accelerometers, RocketSystems* arg) {
    while (true) {
        LowGData lowg = arg->sensors.low_g.read();
        arg->rocket_data.low_g.update(lowg);
        LowGLSM lowglsm = arg->sensors.low_g_lsm.read();
        arg->rocket_data.low_g_lsm.update(lowglsm);
        HighGData highg = arg->sensors.high_g.read();
        arg->rocket_data.high_g.update(highg);
        THREAD_SLEEP(2);
    }
}

/**
 * See \ref data_logger_thread
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
 * See \ref data_logger_thread
 */
DECLARE_THREAD(magnetometer, RocketSystems* arg) {
    while (true) {
        Magnetometer reading = arg->sensors.magnetometer.read();
        arg->rocket_data.magnetometer.update(reading);
        THREAD_SLEEP(50);  //data rate is 155hz so 7 is closest
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(gps, RocketSystems* arg) {
    while (true) {
        GPS reading = arg->sensors.gps.read();
        arg->rocket_data.gps.update(reading);
        THREAD_SLEEP(100);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(voltage, RocketSystems* arg) {
    while (true) {
        Voltage reading = arg->sensors.voltage.read();
        arg->rocket_data.voltage.update(reading);
        THREAD_SLEEP(50);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(continuity, RocketSystems* arg) {
//    Continuity initial_readings = arg->sensors.continuity.read();
//
//    Sound continuity_sensor_tune[] = {
//            { 20, 200 }, { 0, 200 },
//            { initial_readings.sense_apogee ? 40u : 20u, 200 }, { 0, 200 },
//            { 20, 200 }, { 0, 200 }, { 20, 200 }, { 0, 200 },
//            { initial_readings.sense_main ? 40u : 20u, 200 }, { 0, 200 },
//            { 20, 200 }, { 0, 200 }, { 20, 200 }, { 0, 200 }, { 20, 200 }, { 0, 200 },
//            { initial_readings.pins[2] ? 40u : 20u, 200 }, { 0, 200 },
//            { 20, 200 }, { 0, 200 }, { 20, 200 }, { 0, 200 }, { 20, 200 }, { 0, 200 }, { 20, 200 }, { 0, 200 },
//            { initial_readings.pins[3] ? 40u : 20u, 200 }, { 0, 200 },
//    };
//    arg->buzzer.play_tune(continuity_sensor_tune, 28);

    while (true) {
        Continuity reading = arg->sensors.continuity.read();
        arg->rocket_data.continuity.update(reading);

        THREAD_SLEEP(50);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(fsm, RocketSystems* arg) {
    FSM fsm {};
    bool already_played_freebird = false;
    uint16_t ms_per_beat = 1000;

    Sound d4_eight = {294, 0.125 * ms_per_beat};
    Sound g4_eight = {392, 0.125 * ms_per_beat};
    Sound f_nat_4_eight = {350, 0.125 * ms_per_beat};
    Sound b_flat_4_eight = {466, 0.125 * ms_per_beat};
    Sound e4_eight = {330, 0.125 * ms_per_beat};

    Sound d4_quart = {294, 0.25 * ms_per_beat};
    Sound g4_quart = {392, 0.25 * ms_per_beat};
    Sound f_nat_4_quart = {350, 0.25 * ms_per_beat};
    Sound b_flat_4_quart = {466, 0.25 * ms_per_beat};
    Sound e4_quart = {330, 0.25 * ms_per_beat};

    Sound free_bird[] = {d4_eight, g4_eight, d4_eight, f_nat_4_eight, g4_eight, f_nat_4_quart, f_nat_4_quart, f_nat_4_eight, 
                         d4_eight, f_nat_4_eight, f_nat_4_eight, f_nat_4_eight, d4_eight, f_nat_4_quart, f_nat_4_eight, d4_eight, 
                         f_nat_4_eight, f_nat_4_quart, b_flat_4_eight, f_nat_4_quart, b_flat_4_eight, f_nat_4_quart, b_flat_4_eight, 
                         f_nat_4_eight, b_flat_4_eight, f_nat_4_eight, d4_eight, e4_eight, d4_eight, f_nat_4_eight, e4_eight, f_nat_4_quart, 
                         f_nat_4_quart, f_nat_4_eight, d4_eight, f_nat_4_eight, f_nat_4_eight, f_nat_4_eight, d4_eight, f_nat_4_quart, 
                         f_nat_4_eight, d4_eight, f_nat_4_eight, f_nat_4_quart, b_flat_4_eight, f_nat_4_quart, b_flat_4_eight, f_nat_4_quart
                        };

    while (true) {
        FSMState current_state = arg->rocket_data.fsm_state.getRecentUnsync();
        StateEstimate state_estimate(arg->rocket_data);

        FSMState next_state = fsm.tick_fsm(current_state, state_estimate);

        arg->rocket_data.fsm_state.update(next_state);

        if (current_state == FSMState::STATE_SUSTAINER_IGNITION && !already_played_freebird) {
            arg->buzzer.play_tune(free_bird, 48);
            already_played_freebird = true;
        }

        THREAD_SLEEP(50);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(buzzer, RocketSystems* arg) {
    while (true) {
        arg->buzzer.tick();

        THREAD_SLEEP(50);
    }
}

/**
 * See \ref data_logger_thread
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
 * This thread will handle the firing of pyro channels when the FSM sets the pyro flags
*/
DECLARE_THREAD(pyro, RocketSystems* arg) {
    while (true) {
        FSMState current_state = arg->rocket_data.fsm_state.getRecentUnsync();
        PyroState new_pyro_state = arg->sensors.pyro.tick(current_state, arg->rocket_data.orientation.getRecentUnsync());
        arg->rocket_data.pyro.update(new_pyro_state);
        THREAD_SLEEP(200);
    }
}


/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(telemetry_buffering, RocketSystems* arg) {
    while (true) {
        arg->tlm.bufferData(arg->rocket_data);
        THREAD_SLEEP(250/4);
    }
}


/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(telemetry, RocketSystems* arg) {
    while (true) {
        arg->tlm.transmit(arg->rocket_data);
        arg->rocket_data.telem_latency.tick();

        THREAD_SLEEP(1);
    }
}


#define INIT_SYSTEM(s) do { ErrorCode code = (s).init(); if (code != NoError) { return code; } } while (0)
ErrorCode init_systems(RocketSystems& systems) {
    // todo message on failure
    INIT_SYSTEM(systems.sensors.low_g);
    INIT_SYSTEM(systems.sensors.high_g);
    INIT_SYSTEM(systems.sensors.low_g_lsm);
    INIT_SYSTEM(systems.sensors.barometer);
    INIT_SYSTEM(systems.sensors.continuity);
    INIT_SYSTEM(systems.sensors.voltage);
    INIT_SYSTEM(systems.sensors.magnetometer);
    INIT_SYSTEM(systems.sensors.gps);
    INIT_SYSTEM(systems.log_sink);
    INIT_SYSTEM(systems.buzzer);
    INIT_SYSTEM(systems.sensors.pyro);
    INIT_SYSTEM(systems.tlm);
#ifdef IS_SUSTAINER
    INIT_SYSTEM(systems.sensors.orientation);
#endif
    return NoError;
}
#undef INIT_SYSTEM

/**
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

    START_THREAD(logger, DATA_CORE, config, 5);
    START_THREAD(accelerometers, SENSOR_CORE, config, 4);
    START_THREAD(barometer, SENSOR_CORE, config, 4);
    START_THREAD(continuity, SENSOR_CORE, config, 3);
    START_THREAD(voltage, SENSOR_CORE, config, 3);
    START_THREAD(gps, SENSOR_CORE, config, 4);
    START_THREAD(magnetometer, SENSOR_CORE, config, 3);
    START_THREAD(kalman, SENSOR_CORE, config, 4);
    START_THREAD(fsm, SENSOR_CORE, config, 5);
    START_THREAD(buzzer, SENSOR_CORE, config, 1);
    START_THREAD(pyro, SENSOR_CORE, config, 4);
    START_THREAD(telemetry, SENSOR_CORE, config, 5);
    START_THREAD(telemetry_buffering, SENSOR_CORE, config, 3);
#ifdef IS_SUSTAINER
    START_THREAD(orientation, SENSOR_CORE, config, 2);
#endif

    uint16_t ms_per_beat = 1000;

    Sound d4_eight = {294, 0.125 * ms_per_beat};
    Sound g4_eight = {392, 0.125 * ms_per_beat};
    Sound f_nat_4_eight = {350, 0.125 * ms_per_beat};
    Sound b_flat_4_eight = {466, 0.125 * ms_per_beat};
    Sound e4_eight = {330, 0.125 * ms_per_beat};

    Sound d4_quart = {294, 0.25 * ms_per_beat};
    Sound g4_quart = {392, 0.25 * ms_per_beat};
    Sound f_nat_4_quart = {350, 0.25 * ms_per_beat};
    Sound b_flat_4_quart = {466, 0.25 * ms_per_beat};
    Sound e4_quart = {330, 0.25 * ms_per_beat};

    Sound free_bird[] = {d4_eight, g4_eight, d4_eight, f_nat_4_eight, g4_eight, f_nat_4_quart, f_nat_4_quart, f_nat_4_eight, 
                         d4_eight, f_nat_4_eight, f_nat_4_eight, f_nat_4_eight, d4_eight, f_nat_4_quart, f_nat_4_eight, d4_eight, 
                         f_nat_4_eight, f_nat_4_quart, b_flat_4_eight, f_nat_4_quart, b_flat_4_eight, f_nat_4_quart, b_flat_4_eight, 
                         f_nat_4_eight, b_flat_4_eight, f_nat_4_eight, d4_eight, e4_eight, d4_eight, f_nat_4_eight, e4_eight, f_nat_4_quart, 
                         f_nat_4_quart, f_nat_4_eight, d4_eight, f_nat_4_eight, f_nat_4_eight, f_nat_4_eight, d4_eight, f_nat_4_quart, 
                         f_nat_4_eight, d4_eight, f_nat_4_eight, f_nat_4_quart, b_flat_4_eight, f_nat_4_quart, b_flat_4_eight, f_nat_4_quart
                        };
    
    config->buzzer.play_tune(free_bird, 48);
    
    while (true) {
        THREAD_SLEEP(1000);
        Serial.println("running");
    }
}

//void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char* pcTaskName){
//    Serial.println("OVERFLOW");
//    Serial.println((char*)pcTaskName);
//}
