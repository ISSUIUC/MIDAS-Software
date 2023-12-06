#include "systems.h"

#include "hal.h"
#include "sensors.h"
#include "data_logging.h"
#include "buzzer.h"

#include "gnc/example_kf.h"
#include "gnc/displacement_kf.h"

/**
 * These are all the functions that will run in each task
 * Each function has a `while (true)` loop within that should not be returned out of or yielded in any way
 * 
 * @param name the name of the thread, replace with the actual name
 * @param arg the config file for the rocket
 */
DECLARE_THREAD(data_logger, RocketSystems* arg) {
    log_begin(arg->log_sink);
    while (true) {
        log_data(arg->log_sink, arg->rocket_data);
        THREAD_SLEEP(50);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(barometer, RocketSystems* arg) {
    while (true) {
        Barometer reading = arg->sensors.barometer.read();
        arg->rocket_data.barometer.update(reading);
        THREAD_SLEEP(16);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(low_g, RocketSystems* arg) {
    while (true) {
        LowGData reading = arg->sensors.low_g.read();
        arg->rocket_data.low_g.update(reading);
        THREAD_SLEEP(1);
    }
}

DECLARE_THREAD(low_g_lsm, RocketSystems* arg) {
    while (true) {
        LowGLSM reading = arg->sensors.low_g_lsm.read();
        arg->rocket_data.low_g_lsm.update(reading);
        THREAD_SLEEP(10);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(high_g, RocketSystems* arg) {
    while (true) {
        HighGData reading = arg->sensors.high_g.read();
        arg->rocket_data.high_g.update(reading);
        THREAD_SLEEP(1);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(orientation, RocketSystems* arg) {
    while (true) {
        Orientation reading = arg->sensors.orientation.read();
        arg->rocket_data.orientation.update(reading);

        THREAD_SLEEP(16);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(magnetometer, RocketSystems* arg) {
    while (true) {
        Magnetometer reading = arg->sensors.magnetometer.read();
        arg->rocket_data.magnetometer.update(reading);
        THREAD_SLEEP(7);  //data rate is 155hz so 7 is closest
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(gps, RocketSystems* arg) {
    while (true) {
        GPS reading = arg->sensors.gps.read();
        arg->rocket_data.gps.update(reading);
        THREAD_SLEEP(16);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(voltage, RocketSystems* arg) {
    while (true) {
        Voltage reading = arg->sensors.voltage.read();
        arg->rocket_data.voltage.update(reading);
        THREAD_SLEEP(20);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(continuity, RocketSystems* arg) {
    Continuity initial_readings = arg->sensors.continuity.read();

    Sound continuity_sensor_tune[] = {
            { 20, 200 }, { 0, 200 },
            { initial_readings.pins[0] ? 40u : 20u, 200 }, { 0, 200 },
            { 20, 200 }, { 0, 200 }, { 20, 200 }, { 0, 200 },
            { initial_readings.pins[1] ? 40u : 20u, 200 }, { 0, 200 },
            { 20, 200 }, { 0, 200 }, { 20, 200 }, { 0, 200 }, { 20, 200 }, { 0, 200 },
            { initial_readings.pins[2] ? 40u : 20u, 200 }, { 0, 200 },
            { 20, 200 }, { 0, 200 }, { 20, 200 }, { 0, 200 }, { 20, 200 }, { 0, 200 }, { 20, 200 }, { 0, 200 },
            { initial_readings.pins[3] ? 40u : 20u, 200 }, { 0, 200 },
    };
    arg->buzzer.play_tune(continuity_sensor_tune, 28);
    
    while (true) {
        Continuity reading = arg->sensors.continuity.read();
        arg->rocket_data.continuity.update(reading);
        THREAD_SLEEP(16);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(fsm, RocketSystems* arg) {
    while (true) {
         FSMState current_state = arg->rocket_data.fsm_state.getRecent();
         FSMState next_state = tick_fsm(current_state);
         arg->rocket_data.fsm_state.update(next_state);
        THREAD_SLEEP(16);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(buzzer, RocketSystems* arg) {
    while (true) {
        arg->buzzer.tick();

        THREAD_SLEEP(1);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(kalman, RocketSystems* arg) {
    example_kf.initialize();
    TickType_t last = xTaskGetTickCount();

    while (true) { 
        // add the tick update function 
        // FSMState current_state = arg->rocket_state.fsm_state.getRecent();
        // Barometer current_barometer_buf = arg->rocket_state.barometer.getRecent();
        Barometer current_barom_buf = arg->rocket_data.barometer.getRecent();
        LowGData current_accelerometer = arg->rocket_data.low_g.getRecent();
        Acceleration current_accelerations = {
            .ax = current_accelerometer.ax,
            .ay = current_accelerometer.ay,
            .az = current_accelerometer.az
        };
        // HighGData current_accelerometer = arg->rocket_state.high_g.getRecent();
        // Orientation current_orientation = arg->rocket_state.orientation.getRecent();
        // example_kf.kfTickFunction(current_state.curr_state, current_barometer_buf, current_accelerometer, current_orientation ,xTaskGetTickCount() - last);
        float dt = (xTaskGetTickCount() - last);
        displacement_kf.kfTick(dt, 13.0, current_barom_buf, current_accelerations);

        last = xTaskGetTickCount(); // THREAD_SLEEP(16);

        //Serial.println("KALMAN");
        
        THREAD_SLEEP(16);
    }
}

#define INIT_SYSTEM(s) do { ErrorCode code = (s).init(); if (code != NoError) { return false; } } while (0)
bool init_systems(RocketSystems& systems) {
    // todo message on failure
    INIT_SYSTEM(systems.sensors.low_g);
    INIT_SYSTEM(systems.sensors.high_g);
    INIT_SYSTEM(systems.sensors.low_g_lsm);
    INIT_SYSTEM(systems.sensors.barometer);
    INIT_SYSTEM(systems.sensors.continuity);
    INIT_SYSTEM(systems.sensors.orientation);
    INIT_SYSTEM(systems.sensors.voltage);
    INIT_SYSTEM(systems.sensors.magnetometer);
    INIT_SYSTEM(systems.sensors.gps);
    INIT_SYSTEM(systems.log_sink);
    INIT_SYSTEM(systems.buzzer);

    return true;
}
#undef INIT_SYSTEM

/**
 * Creates all threads for each sensor, FSM, Kalman algorithm, and data logging member
 * Starts thread scheduler to actually start doing jobs
*/
void begin_systems(RocketSystems* config) {
    bool success = init_systems(*config);
    if (!success) {
        // todo some message probably
        return;
    }
    START_THREAD(data_logger, DATA_CORE, config);
    START_THREAD(barometer, SENSOR_CORE, config);
    START_THREAD(low_g, SENSOR_CORE, config);
    START_THREAD(high_g, SENSOR_CORE, config);
    START_THREAD(low_g_lsm, SENSOR_CORE, config);
    START_THREAD(orientation, SENSOR_CORE, config);
    START_THREAD(magnetometer, SENSOR_CORE, config);
    START_THREAD(gps, DATA_CORE, config);
    START_THREAD(voltage, SENSOR_CORE, config);
    START_THREAD(continuity, SENSOR_CORE, config);
    START_THREAD(fsm, SENSOR_CORE, config);
    START_THREAD(buzzer, SENSOR_CORE, config);
    START_THREAD(kalman, SENSOR_CORE, config);

    while (true) {
        THREAD_SLEEP(1000);
    }
}
