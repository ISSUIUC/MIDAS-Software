#include "systems.h"

#include "hal.h"
#include "sensors.h"
#include "data_logging.h"
#include "buzzer.h"

#include "gnc/displacement_kf.h"

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
        Serial.println("Entered Logger");

        log_data(arg->log_sink, arg->rocket_data);
        THREAD_SLEEP(50);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(barometer, RocketSystems* arg) {
    while (true) {
//        Serial.println("Entered Barometer");

        Barometer reading = arg->sensors.barometer.read();
        arg->rocket_data.barometer.update(reading);

        Serial.print("Baro Alt: ");
        Serial.print(reading.altitude);
        Serial.print(" Pressure: ");
        Serial.print(reading.pressure);
        Serial.print(" Temp: ");
        Serial.print(reading.temperature);
        Serial.print("\n");
        THREAD_SLEEP(16);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(low_g, RocketSystems* arg) {
    while (true) {
//        Serial.println("Entered Low G");

        LowGData reading = arg->sensors.low_g.read();
        arg->rocket_data.low_g.update(reading);

//        Serial.print(" Ax: ");
//        Serial.print(reading.ax);
//        Serial.print(" Ay: ");
//        Serial.print(reading.ay);
//        Serial.print(" Az: ");
//        Serial.print(reading.az);
//        Serial.print("\n");
        THREAD_SLEEP(1);
    }
}

DECLARE_THREAD(low_g_lsm, RocketSystems* arg) {
    while (true) {
//        Serial.println("Entered LowG LSM");

        LowGLSM reading = arg->sensors.low_g_lsm.read();
        arg->rocket_data.low_g_lsm.update(reading);

//        Serial.print("gx: ");
//        Serial.print(reading.gx);
//        Serial.print(" gy: ");
//        Serial.print(reading.gy);
//        Serial.print(" gz: ");
//        Serial.print(reading.gz);
//        Serial.print("\n");
        THREAD_SLEEP(10);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(high_g, RocketSystems* arg) {
    while (true) {
//        Serial.println("Entered High G");

        HighGData reading = arg->sensors.high_g.read();
        arg->rocket_data.high_g.update(reading);
//        Serial.print("Ax: ");
//        Serial.print(reading.ax);
//        Serial.print(" Ay: ");
//        Serial.print(reading.ay);
//        Serial.print(" Az: ");
//        Serial.print(reading.az);
//        Serial.print("\n");
        THREAD_SLEEP(1);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(orientation, RocketSystems* arg) {
    while (true) {
//        Serial.println("Entered Orientation");

        Orientation reading = arg->sensors.orientation.read();
        if (reading.has_data) {
            arg->rocket_data.orientation.update(reading);

//            Serial.print(" Yaw: ");
//            Serial.print(reading.yaw);
//            Serial.print(" Pitch: ");
//            Serial.print(reading.pitch);
//            Serial.print(" Roll: ");
//            Serial.print(reading.roll);
//            Serial.print("\n");
        }

        THREAD_SLEEP(500);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(magnetometer, RocketSystems* arg) {
    while (true) {
//        Serial.println("Entered Magnetometer");

        Magnetometer reading = arg->sensors.magnetometer.read();
        arg->rocket_data.magnetometer.update(reading);

//        Serial.print(" mx: ");
//        Serial.print(reading.mx);
//        Serial.print(" my: ");
//        Serial.print(reading.my);
//        Serial.print(" mz: ");
//        Serial.print(reading.mz);
//        Serial.print("\n");
        THREAD_SLEEP(7);  //data rate is 155hz so 7 is closest
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(gps, RocketSystems* arg) {
    while (true) {
//        Serial.println("Entered GPS");

        GPS reading = arg->sensors.gps.read();
        arg->rocket_data.gps.update(reading);

//        Serial.print(" Satellite count: ");
//        Serial.print(reading.satellite_count);
//        Serial.print(" Alt: ");
//        Serial.print(reading.altitude);
//        Serial.print("\n");
        THREAD_SLEEP(16);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(voltage, RocketSystems* arg) {
    while (true) {
//        Serial.println("Entered Voltage");

        Voltage reading = arg->sensors.voltage.read();
        arg->rocket_data.voltage.update(reading);
//        Serial.print(" Voltage: ");
//        Serial.print(reading.voltage);
//        Serial.print("\n");
        THREAD_SLEEP(20);
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
//        Serial.println("Entered Continuity");

        Continuity reading = arg->sensors.continuity.read();
        arg->rocket_data.continuity.update(reading);

//        Serial.print(reading.sense_pyro);
//        Serial.print(reading.pins[0]);
//        Serial.print(reading.pins[1]);
//        Serial.print(reading.pins[2]);
//        Serial.print(reading.pins[3]);

        THREAD_SLEEP(16);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(fsm, RocketSystems* arg) {
    FSM fsm {};

    while (true) {
//        Serial.println("Entered FSM");

        FSMState current_state = arg->rocket_data.fsm_state.getRecent();
        StateEstimate state_estimate(arg->rocket_data);

        FSMState next_state = fsm.tick_fsm(current_state, state_estimate);

//        Serial.print("Est altitude: ");
//        Serial.print(state_estimate.altitude);
//        Serial.print(" Est jerk: ");
//        Serial.print(state_estimate.jerk);
//        Serial.print("\n");

        arg->rocket_data.fsm_state.update(next_state);
        THREAD_SLEEP(16);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(buzzer, RocketSystems* arg) {
    while (true) {
//        Serial.println("Entered Buzzer");

        arg->buzzer.tick();

        THREAD_SLEEP(1);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(kalman, RocketSystems* arg) {
    displacement_kf.initialize();
    TickType_t last = xTaskGetTickCount();

    while (true) {
//        Serial.println("Entered Kalman");

        // add the tick update function
        Barometer current_barom_buf = arg->rocket_data.barometer.getRecent();
        LowGData current_accelerometer = arg->rocket_data.low_g.getRecent();
        Acceleration current_accelerations = {
            .ax = current_accelerometer.ax,
            .ay = current_accelerometer.ay,
            .az = current_accelerometer.az
        };
        float dt = pdTICKS_TO_MS(xTaskGetTickCount() - last) / 1000.0f;
        displacement_kf.kfTick(dt, 13.0, current_barom_buf, current_accelerations);
        KalmanData current_state = displacement_kf.getState();

        arg->rocket_data.kalman.update(current_state);

        last = xTaskGetTickCount();

        THREAD_SLEEP(16);
    }
}

/**
 * This thread will handle the firing of pyro channels when the FSM sets the pyro flags
*/
DECLARE_THREAD(pyro, RocketSystems* arg) {
    while (true) {
//        Serial.println("Entered Pyro");

        FSMState current_state = arg->rocket_data.fsm_state.getRecent();
        PyroState new_pyro_state = arg->sensors.pyro.tick(current_state, arg->rocket_data.orientation.getRecent());
        arg->rocket_data.pyro.update(new_pyro_state);
        THREAD_SLEEP(16);
    }
}


/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(telemetry_buffering, RocketSystems* arg) {
    while (true) {
        Serial.println("Entered Telem Buffering");

        arg->tlm.bufferData(arg->rocket_data);
        THREAD_SLEEP(5);
    }
}


/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(telemetry, RocketSystems* arg) {
    while (true) {
        Serial.println("Entered Telem Transmit");

        arg->tlm.transmit(arg->rocket_data);

        Serial.println("Exit Telem Transmit");
        THREAD_SLEEP(16);
    }
}

#define INIT_SYSTEM(s) do { ErrorCode code = (s).init(); if (code != NoError) { return code; } } while (0)
ErrorCode init_systems(RocketSystems& systems) {
    // todo message on failure
//    INIT_SYSTEM(systems.sensors.low_g);
//    INIT_SYSTEM(systems.sensors.high_g);
//    INIT_SYSTEM(systems.sensors.low_g_lsm);
    INIT_SYSTEM(systems.sensors.barometer);
//    INIT_SYSTEM(systems.sensors.continuity);
//    INIT_SYSTEM(systems.sensors.orientation);
//    INIT_SYSTEM(systems.sensors.voltage);
//    INIT_SYSTEM(systems.sensors.magnetometer);
//    INIT_SYSTEM(systems.sensors.gps);
    INIT_SYSTEM(systems.log_sink);
//    INIT_SYSTEM(systems.buzzer);
//    INIT_SYSTEM(systems.sensors.pyro);
//    INIT_SYSTEM(systems.tlm);
    return NoError;
}
#undef INIT_SYSTEM

/**
 * Creates all threads for each sensor, FSM, Kalman algorithm, and data logging member
 * Starts thread scheduler to actually start doing jobs
*/
void begin_systems(RocketSystems* config) {
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

    START_THREAD(logger, SENSOR_CORE, config);
    START_THREAD(barometer, DATA_CORE, config);
//    START_THREAD(low_g, SENSOR_CORE, config);
//    START_THREAD(high_g, SENSOR_CORE, config);
//    START_THREAD(low_g_lsm, SENSOR_CORE, config);
//    START_THREAD(orientation, SENSOR_CORE, config);
//    START_THREAD(magnetometer, SENSOR_CORE, config);
//    START_THREAD(gps, SENSOR_CORE, config);
//    START_THREAD(voltage, SENSOR_CORE, config);
//    START_THREAD(continuity, SENSOR_CORE, config);
//    START_THREAD(fsm, SENSOR_CORE, config);
//    START_THREAD(buzzer, SENSOR_CORE, config);
//    START_THREAD(kalman, SENSOR_CORE, config);
//    START_THREAD(pyro, SENSOR_CORE, config);
//    START_THREAD(telemetry, DATA_CORE, config);
//    START_THREAD(telemetry_buffering, DATA_CORE, config);

    while (true) {
        THREAD_SLEEP(1000);
    }
}
