#include "systems.h"
#include "data_logging.h"
#include "finite-state-machines/fsm.h"
#include "gnc/yessir.h"

/**
 * The size of a thread stack, in bytes.
 */
#define STACK_SIZE 8192

/**
 * The ESP32 has two physical cores, which will each be dedicated to one group of threads.
 * The SENSOR_CORE runs the threads which write to the sensor_data struct (mostly sensor polling threads).
 */
#define SENSOR_CORE ((BaseType_t) 0)

/**
 * The ESP32 has two physical cores, which will each be dedicated to one group of threads.
 * The DATA_CORE runs the GPS thread, as well as the threads which read from the sensor_data struct (e.g. SD logging).
 */
#define DATA_CORE ((BaseType_t) 1)

/**
 * Macro for declaring a thread. Creates a function with the name suffixed with `_thread`, annotated with [[noreturn]].
 *
 * @param name The name of the task.
 * @param param The single parameter to input into the thread.
 */
#define DECLARE_THREAD(name, param) [[noreturn]] void name##_thread(param)

/**
 * Macro for creating and starting a thread declared with `DECLARE_THREAD`. This is a statement and has no return value.
 * Never put this in a scope that will end before the thread should.
 *
 * @param name Name of the thread.
 * @param core Core for the task to be pinned to, either `SENSOR_CORE` or `DATA_CORE`.
 * @param arg Argument passed in to the `param` argument of `DECLARE_THREAD`.
 * @param prio Priority of the thread.
 */
#define START_THREAD(name, core, arg, prio) StaticTask_t name##_task;                \
                                      static unsigned char name##_stack[STACK_SIZE];            \
                                      xTaskCreateStaticPinnedToCore(((TaskFunction_t) name##_thread), #name, STACK_SIZE, arg, tskIDLE_PRIORITY + (prio), name##_stack, &name##_task, core)
/*
 * Parameters for xTaskCreateStaticPinnedToCore are as follows in parameter order:
 *  - Function to be run by the thread, this contains a `while(true)` loop
 *  - Name of thread
 *  - Size of the stack for each thread in words (1 word = 4 bytes)
 *  - Arguments to be passed into the function, this will generally eb the config file
 *  - Priority of the task, in allmost all cases, this will be the idle priority plus one
 *  - The actual stack memory to use
 *  - A handle to reference the task with
 *  - The core to pin the task to
 */

/**
 * @brief These are all the functions that will run in each task
 * Each function has a `while (true)` loop within that should not be returned out of or yielded in any way
 *
 * The `DECLARE_THREAD` macro creates a function whose name is suffixed by _thread, and annotates it with [[noreturn]]
 */
DECLARE_THREAD(logger, RocketSystems* arg) {
    log_begin(arg->sensors.sink);
    while (true) {
        log_data(arg->sensors.sink, arg->rocket_data);

        arg->rocket_data.log_latency.tick();

        THREAD_SLEEP(1);
    }
}

DECLARE_THREAD(barometer, RocketSystems* arg) {
    // Reject single rogue barometer readings that are very different from the immediately prior reading
    // Will only reject a certain number of readings in a row
    BarometerData prev_reading;
    constexpr float altChgThreshold = 200; // meters
    constexpr float presChgThreshold = 500; // milibars
    constexpr float tempChgThreshold = 10; // degrees C
    constexpr unsigned int maxConsecutiveRejects = 3;
    unsigned int rejects = maxConsecutiveRejects; // Always accept first reading
    while (true) {
        BarometerData reading = arg->sensors.barometer.read();
        bool is_rogue = std::abs(prev_reading.altitude - reading.altitude) > altChgThreshold;
                        //std::abs(prev_reading.pressure - reading.pressure) > presChgThreshold ||
                        //std::abs(prev_reading.temperature - reading.temperature) > tempChgThreshold;
        // TODO: Log when we receive a rejection!
        if (is_rogue && rejects++ < maxConsecutiveRejects)
            arg->rocket_data.barometer.update(prev_reading); // Reuse old reading, reject new reading
        else {
            rejects = 0;
            arg->rocket_data.barometer.update(reading);
            prev_reading = reading; // Only update prev_reading with accepted readings
        }
        THREAD_SLEEP(6);
    }
}

DECLARE_THREAD(accelerometers, RocketSystems* arg) {
    while (true) {
#ifdef IS_SUSTAINER
        LowGData lowg = arg->sensors.low_g.read();
        arg->rocket_data.low_g.update(lowg);
#endif
        LowGLSMData lowglsm = arg->sensors.low_g_lsm.read();
        arg->rocket_data.low_g_lsm.update(lowglsm);
        HighGData highg = arg->sensors.high_g.read();
        arg->rocket_data.high_g.update(highg);
        THREAD_SLEEP(2);
    }
}

DECLARE_THREAD(orientation, RocketSystems* arg) {
    while (true) {
        OrientationData reading = arg->sensors.orientation.read();
        if (reading.has_data) {
            arg->rocket_data.orientation.update(reading);
        }

        THREAD_SLEEP(100);
    }
}

DECLARE_THREAD(magnetometer, RocketSystems* arg) {
    while (true) {
        MagnetometerData reading = arg->sensors.magnetometer.read();
        arg->rocket_data.magnetometer.update(reading);
        THREAD_SLEEP(50);  //data rate is 155hz so 7 is closest
    }
}

// Ever device which communicates over i2c is on this thread to avoid interference
DECLARE_THREAD(i2c, RocketSystems* arg) {
    int i = 0;

    while (true) {
        if (i % 10 == 0) {
            GPSData reading = arg->sensors.gps.read();
            arg->rocket_data.gps.update(reading);

            FSMState current_state = arg->rocket_data.fsm_state.getRecentUnsync();
            PyroState new_pyro_state = arg->pyro.tick_pyro(current_state, arg->rocket_data.orientation.getRecentUnsync());
            arg->rocket_data.pyro.update(new_pyro_state);

            ContinuityData reading2 = arg->sensors.continuity.read();
            arg->rocket_data.continuity.update(reading2);

            VoltageData reading3 = arg->sensors.voltage.read();
            arg->rocket_data.voltage.update(reading3);
        }

        arg->led.update();
        i += 1;

        THREAD_SLEEP(10);
    }
}

// This thread has a bit of extra logic since it needs to play a tune exactly once the sustainer ignites
DECLARE_THREAD(fsm, RocketSystems* arg) {
    FSM fsm{};
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

DECLARE_THREAD(buzzer, RocketSystems* arg) {
    while (true) {
        arg->buzzer.tick();

        THREAD_SLEEP(10);
    }
}

DECLARE_THREAD(kalman, RocketSystems* arg) {
    // Orientation initial_orientation = arg->rocket_data.orientation.getRecent();
    // Barometer initial_barom_buf = arg->rocket_data.barometer.getRecent();
    // LowGData initial_accelerometer = arg->rocket_data.low_g.getRecent();
    //yessir.initialize(initial_orientation, initial_barom_buf, initial_accelerations);
    TickType_t last = 0;
    
    while (true) {
        if (yessir.should_reinit) {
            yessir.initialize(arg->rocket_data);
            last = xTaskGetTickCount();
            yessir.should_reinit = false;
        }
        // add the tick update function
        BarometerData current_barom_buf = arg->rocket_data.barometer.getRecent();
        OrientationData current_orientation = arg->rocket_data.orientation.getRecent();
        HighGData current_accelerometer = arg->rocket_data.high_g.getRecent();
        FSMState FSM_state = arg->rocket_data.fsm_state.getRecent();
        Acceleration current_accelerations = {
            .ax = current_accelerometer.ax,
            .ay = current_accelerometer.ay,
            .az = current_accelerometer.az
        };
        float dt = pdTICKS_TO_MS(xTaskGetTickCount() - last) / 1000.0f;

        yessir.tick(dt, 13.0, current_barom_buf, current_accelerations, current_orientation, FSM_state);
        KalmanData current_state = yessir.getState();

        arg->rocket_data.kalman.update(current_state);

        last = xTaskGetTickCount();

        THREAD_SLEEP(50);
    }
}

DECLARE_THREAD(telemetry, RocketSystems* arg) {
    while (true) {
        arg->tlm.transmit(arg->rocket_data, arg->led);

        if (arg->rocket_data.fsm_state.getRecentUnsync() == STATE_IDLE) {
            TelemetryCommand command;
            if (arg->tlm.receive(&command, 2000)) {
                if(command.valid()) {
                    arg->tlm.acknowledgeReceived();
                    switch(command.command) {
                        case CommandType::RESET_KF:
                            yessir.should_reinit = true;
                            break;
                        default:
                            break; 
                    }
                }

            }
        }
        THREAD_SLEEP(1);
    }
}

#define INIT_SYSTEM(s) do { ErrorCode code = (s).init(); if (code != NoError) { return code; } } while (0)

/**
 * @brief Initializes all systems in order, returning early if a system's initialization process errors out.
 *        Turns on the Orange LED while initialization is running.
 */
ErrorCode RocketSystems::init_systems() {
    INIT_SYSTEM(led);
    sensors.led.set(LED::ORANGE, true);

    INIT_SYSTEM(sensors.sink);
    INIT_SYSTEM(sensors.high_g);
    INIT_SYSTEM(sensors.low_g_lsm);
    INIT_SYSTEM(sensors.barometer);
    INIT_SYSTEM(sensors.magnetometer);
    INIT_SYSTEM(sensors.continuity);
    INIT_SYSTEM(sensors.voltage);
    INIT_SYSTEM(pyro);
    INIT_SYSTEM(buzzer);
    INIT_SYSTEM(tlm);
    INIT_SYSTEM(sensors.gps);

#ifdef IS_SUSTAINER
    INIT_SYSTEM(sensors.low_g);
    INIT_SYSTEM(sensors.orientation);
#endif

    sensors.led.set(LED::ORANGE, false);
    return NoError;
}
#undef INIT_SYSTEM


RocketSystems::RocketSystems(Sensors hardware) : sensors(hardware), buzzer(hardware.buzzer), led(hardware.led),
                                                 tlm(hardware.telemetry), pyro(hardware.pyro) {
}

/**
 * @brief Initializes the systems, and then creates and starts the thread for each system.
 *        If initialization fails, then this enters an infinite loop.
 */
[[noreturn]] void RocketSystems::begin() {
    Serial.println("Starting Systems...");
    ErrorCode init_error_code = init_systems();
    if (init_error_code != NoError) {
        // todo some message probably
        while (true) {
            Serial.print("Had Error: ");
            Serial.print(init_error_code);
            Serial.print("\n");
            Serial.flush();
            update_error_LED(sensors.led, init_error_code);
        }
    }

#ifdef IS_SUSTAINER
    START_THREAD(orientation, SENSOR_CORE, this, 10);
#endif

    START_THREAD(logger, DATA_CORE, this, 15);
    START_THREAD(accelerometers, SENSOR_CORE, this, 13);
    START_THREAD(barometer, SENSOR_CORE, this, 12);
    START_THREAD(i2c, SENSOR_CORE, this, 9);
    START_THREAD(magnetometer, SENSOR_CORE, this, 11);
    START_THREAD(kalman, SENSOR_CORE, this, 7);
    START_THREAD(fsm, SENSOR_CORE, this, 8);
    START_THREAD(buzzer, SENSOR_CORE, this, 6);
    START_THREAD(telemetry, SENSOR_CORE, this, 15);

    buzzer.play_tune(free_bird, FREE_BIRD_LENGTH);

    while (true) {
        THREAD_SLEEP(1000);
        Serial.print("Running (Log Latency: ");
        Serial.print(rocket_data.log_latency.getLatency());
        Serial.println(")");
    }
}

//void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char* pcTaskName){
//    Serial.println("OVERFLOW");
//    Serial.println((char*)pcTaskName);
//}
