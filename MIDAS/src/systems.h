#pragma once

#include "data_logging.h"
#include "buzzer.h"
#include "led.h"
#include "telemetry.h"
#include "b2b_interface.h"
#include "finite-state-machines/fsm.h"
#include "gnc/ekf.h"
#include "pyro.h"

#define ENABLE_TELEM

#if defined(IS_SUSTAINER) && defined(IS_BOOSTER)
#error "Only one of IS_SUSTAINER and IS_BOOSTER may be defined at the same time."
#elif !defined(IS_SUSTAINER) && !defined(IS_BOOSTER)
#error "At least one of IS_SUSTAINER and IS_BOOSTER must be defined."
#endif

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
 * Macro for declaring a thread. Creates a method with the name suffixed with `_thread`, annotated with [[noreturn]].
 *
 * @param name The name of the task.
 */
#define DECLARE_THREAD(name) unsigned char name##_stack[STACK_SIZE] = {}; [[noreturn]] void name##_thread()

#define DEFINE_THREAD(name) template<typename Hw> [[noreturn]] void RocketSystems<Hw>::name##_thread()

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
                                      xTaskCreateStaticPinnedToCore(((TaskFunction_t) [](void* sys) { ((RocketSystems<Hw>*) sys)->name##_thread(); }), #name, STACK_SIZE, arg, tskIDLE_PRIORITY + prio, name##_stack, &name##_task, core)
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
 * @struct RocketData
 * 
 * @brief holds all information about the rocket, sensors, and controllers
 */
template<typename Hw>
struct RocketSystems {
    HwInterface<Hw>& hw;
    RocketData rocket_data;
    LogSink& log_sink;
    BuzzerController buzzer;
    LEDController led;
    PyroLogic<Hw> pyro;
    Telemetry tlm;
    B2BInterface b2b;
    EKF ekf;

    RocketSystems(HwInterface<Hw>& hw, LogSink& log_sink) : hw(hw), log_sink(log_sink), pyro(hw) { }

    ErrorCode init_systems();
    [[noreturn]] void begin();

    void handle_tlm_command(TelemetryCommand& command, FSMState current_state);

    DECLARE_THREAD(logger);
    DECLARE_THREAD(barometer);
    DECLARE_THREAD(accelerometers);
    DECLARE_THREAD(orientation);
    DECLARE_THREAD(magnetometer);
    DECLARE_THREAD(gps);
    DECLARE_THREAD(pyro);
    DECLARE_THREAD(voltage);
    DECLARE_THREAD(fsm);
    DECLARE_THREAD(buzzer);
    DECLARE_THREAD(kalman);
    DECLARE_THREAD(cam);
    DECLARE_THREAD(telemetry);
};

/**
 * @brief These are all the functions that will run in each task
 * Each function has a `while (true)` loop within that should not be returned out of or yielded in any way
 *
 * The `DECLARE_THREAD` macro creates a function whose name is suffixed by _thread, and annotates it with [[noreturn]]
 */

DEFINE_THREAD(logger) {
    log_begin(log_sink);
    while (true) {
        log_data(log_sink, rocket_data);

        rocket_data.log_latency.tick();

        THREAD_SLEEP(1);
    }
}

DEFINE_THREAD(barometer) {
    // Reject single rogue barometer readings that are very different from the immediately prior reading
    // Will only reject a certain number of readings in a row
    BarometerData prev_reading;
    constexpr float altChgThreshold = 200; // meters
    constexpr float presChgThreshold = 500; // milibars
    constexpr float tempChgThreshold = 10; // degrees C
    constexpr unsigned int maxConsecutiveRejects = 3;
    unsigned int rejects = maxConsecutiveRejects; // Always accept first reading
    while (true) {
        BarometerData reading = hw.read_barometer();
        bool is_rogue = std::abs(prev_reading.altitude - reading.altitude) > altChgThreshold;
        //std::abs(prev_reading.pressure - reading.pressure) > presChgThreshold ||
        //std::abs(prev_reading.temperature - reading.temperature) > tempChgThreshold;
        // TODO: Log when we receive a rejection!
        if (is_rogue && rejects++ < maxConsecutiveRejects)
            rocket_data.barometer.update(prev_reading); // Reuse old reading, reject new reading
        else {
            rejects = 0;
            rocket_data.barometer.update(reading);
            prev_reading = reading; // Only update prev_reading with accepted readings
        }
        // Serial.print("Barometer ");
        // Serial.print(reading.altitude);
        // Serial.print(" ");
        // Serial.print(reading.pressure);
        // Serial.print(" ");
        // Serial.println(reading.temperature);
        THREAD_SLEEP(6);
    }
}

DEFINE_THREAD(accelerometers) {
    while (true) {
        LowGData lowg = hw.read_low_g();
        rocket_data.low_g.update(lowg);
        LowGLSMData lowglsm = hw.read_low_g_lsm();
        rocket_data.low_g_lsm.update(lowglsm);
        HighGData highg = hw.read_high_g();
        rocket_data.high_g.update(highg);

        THREAD_SLEEP(2);
    }
}

DEFINE_THREAD(orientation) {
    while (true) {
        OrientationData orientation_holder = rocket_data.orientation.getRecent();
        OrientationData reading = hw.read_orientation();
        if (reading.has_data) {
            if (reading.reading_type == OrientationReadingType::ANGULAR_VELOCITY_UPDATE) {
                orientation_holder.angular_velocity.vx = reading.angular_velocity.vx;
                orientation_holder.angular_velocity.vy = reading.angular_velocity.vy;
                orientation_holder.angular_velocity.vz = reading.angular_velocity.vz;
            } else {
                float old_vx = orientation_holder.angular_velocity.vx;
                float old_vy = orientation_holder.angular_velocity.vy;
                float old_vz = orientation_holder.angular_velocity.vz;
                orientation_holder = reading;
                orientation_holder.angular_velocity.vx = old_vx;
                orientation_holder.angular_velocity.vy = old_vy;
                orientation_holder.angular_velocity.vz = old_vz;
            }

            rocket_data.orientation.update(orientation_holder);
        }

        THREAD_SLEEP(100);
    }
}

DEFINE_THREAD(magnetometer) {
    while (true) {
        MagnetometerData reading = hw.read_magnetometer();
        rocket_data.magnetometer.update(reading);
        THREAD_SLEEP(50);
    }
}

DEFINE_THREAD(gps) {
    while (true) {
        if (hw.is_gps_ready()) {
            GPSData reading = hw.read_gps();
            rocket_data.gps.update(reading);
        }
        //GPS waits internally
        THREAD_SLEEP(1);
    }
}

DEFINE_THREAD(pyro) {
    while (true) {
        FSMState current_state = rocket_data.fsm_state.getRecentUnsync();
        CommandFlags& command_flags = rocket_data.command_flags;

        PyroState new_pyro_state = pyro.tick(current_state, rocket_data.orientation.getRecentUnsync(), command_flags);
        rocket_data.pyro.update(new_pyro_state);

        led.update();

        THREAD_SLEEP(10);
    }
}

DEFINE_THREAD(voltage) {
    while (true) {
        ContinuityData reading = hw.read_continuity();
        VoltageData reading2 = hw.read_voltage();

        rocket_data.continuity.update(reading);
        rocket_data.voltage.update(reading2);

        THREAD_SLEEP(100);
    }
}

// This thread has a bit of extra logic since it needs to play a tune exactly once the sustainer ignites
DEFINE_THREAD(fsm) {
    FSM fsm {};
    bool already_played_freebird = false;
    double last_time_led_flash = pdTICKS_TO_MS(xTaskGetTickCount());
    while (true) {
        FSMState current_state = rocket_data.fsm_state.getRecentUnsync();
        StateEstimate state_estimate(rocket_data);
        CommandFlags& telemetry_commands = rocket_data.command_flags;
        double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

        FSMState next_state = fsm.tick_fsm(current_state, state_estimate, telemetry_commands);

        rocket_data.fsm_state.update(next_state);

        if (current_state == FSMState::STATE_SAFE) {
            if ((current_time - last_time_led_flash) > 250) {
                // Flashes green LED at 4Hz while in SAFE mode.
                last_time_led_flash = current_time;
                led.toggle(LED::GREEN);
            }
        } else {
            led.set(LED::GREEN, LOW);
        }

        if ((current_state == FSMState::STATE_PYRO_TEST || current_state == FSMState::STATE_IDLE) &&
            !buzzer.is_playing()) {
            buzzer.play_tune(warn_tone, WARN_TONE_LENGTH);
        }

        if (current_state == FSMState::STATE_LANDED && !buzzer.is_playing()) {
            buzzer.play_tune(land_tone, LAND_TONE_LENGTH);
        }

        if (current_state == FSMState::STATE_SUSTAINER_IGNITION && !already_played_freebird) {
            buzzer.play_tune(free_bird, FREE_BIRD_LENGTH);
            already_played_freebird = true;
        }

        // FSM-based camera control
        if (rocket_data.command_flags.FSM_should_set_cam_feed_cam1) {
            // Swap camera feed to MUX 1 (Side-facing camera) at launch.
            rocket_data.command_flags.FSM_should_set_cam_feed_cam1 = false;
            b2b.camera.vmux_set(SIDE_CAMERA);
        }

        if (rocket_data.command_flags.FSM_should_swap_camera_feed) {
            // Swap camera feed to MUX 2 (recovery bay camera)
            rocket_data.command_flags.FSM_should_swap_camera_feed = false;
            b2b.camera.vmux_set(BULKHEAD_CAMERA);
        }

        THREAD_SLEEP(50);
    }
}

DEFINE_THREAD(buzzer) {
    while (true) {
        buzzer.tick();

        THREAD_SLEEP(10);
    }
}

DEFINE_THREAD(kalman) {
    ekf.initialize(rocket_data);
    TickType_t last = xTaskGetTickCount();
    // Serial.println("Initialized ekf :(");

    while (true) {
        if (rocket_data.command_flags.should_reset_kf) {
            ekf.initialize(rocket_data);
            last = xTaskGetTickCount();
            rocket_data.command_flags.should_reset_kf = false;
        }

        // add the tick update function
        BarometerData current_barom_buf = rocket_data.barometer.getRecent();
        OrientationData current_orientation = rocket_data.orientation.getRecent();
        HighGData current_accelerometer = rocket_data.high_g.getRecent();
        FSMState FSM_state = rocket_data.fsm_state.getRecent();
        Acceleration current_accelerations = {
            .ax = current_accelerometer.ax,
            .ay = current_accelerometer.ay,
            .az = current_accelerometer.az
        };

        TickType_t now = xTaskGetTickCount();
        float dt = pdTICKS_TO_MS(now - last) / 1000.0f;
        last = now;

        ekf.tick(dt, 13.0, current_barom_buf, current_accelerations, current_orientation, FSM_state);
        KalmanData current_state = ekf.getState();

        rocket_data.kalman.update(current_state);

        // Serial.println("Kalman");
        THREAD_SLEEP(50);
    }
}

template<typename Hw>
void RocketSystems<Hw>::handle_tlm_command(TelemetryCommand& command, FSMState current_state) {
    // maybe we should move this somewhere else but it can stay here for now
    switch (command.command) {
        case CommandType::RESET_KF: {
            rocket_data.command_flags.should_reset_kf = true;
            break;
        }
        case CommandType::SWITCH_TO_SAFE: {
            rocket_data.command_flags.should_transition_safe = true;
            break;
        }
        case CommandType::SWITCH_TO_PYRO_TEST: {
            rocket_data.command_flags.should_transition_pyro_test = true;
            Serial.println("Changing to pyro test");
            break;
        }
        case CommandType::SWITCH_TO_IDLE: {
            rocket_data.command_flags.should_transition_idle = true;
            break;
        }
        case CommandType::FIRE_PYRO_A: {
            if (current_state == FSMState::STATE_PYRO_TEST) {
                rocket_data.command_flags.should_fire_pyro_a = true;
            }
            break;
        }
        case CommandType::FIRE_PYRO_B: {
            if (current_state == FSMState::STATE_PYRO_TEST) {
                rocket_data.command_flags.should_fire_pyro_b = true;
            }
            break;
        }
        case CommandType::FIRE_PYRO_C: {
            if (current_state == FSMState::STATE_PYRO_TEST) {
                rocket_data.command_flags.should_fire_pyro_c = true;
            }
            break;
        }
        case CommandType::FIRE_PYRO_D: {
            if (current_state == FSMState::STATE_PYRO_TEST) {
                rocket_data.command_flags.should_fire_pyro_d = true;
            }
            break;
        }
        case CommandType::CAM_ON: {
            b2b.camera.camera_on(CAM_1);
            b2b.camera.camera_on(CAM_2);
            b2b.camera.vtx_on();
            break;
        }
        case CommandType::CAM_OFF: {
            b2b.camera.camera_off(CAM_1);
            b2b.camera.camera_off(CAM_2);
            b2b.camera.vtx_off();
            break;
        }
        case CommandType::TOGGLE_CAM_VMUX: {
            b2b.camera.vmux_toggle();
            break;
        }
        default: {
            break; // how
        }
    }
}

DEFINE_THREAD(cam) {
    while (true) {
        rocket_data.camera_state = b2b.camera.read();
        THREAD_SLEEP(200);
    }
}

DEFINE_THREAD(telemetry) {
    double launch_time = 0;
    bool has_triggered_vmux_fallback = false;

    while (true) {

        tlm.transmit(rocket_data, led);

        FSMState current_state = rocket_data.fsm_state.getRecentUnsync();
        double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

        if (current_state == FSMState::STATE_IDLE) {
            launch_time = current_time;
            has_triggered_vmux_fallback = false;
        }

        if ((current_time - launch_time) > 80000 && !has_triggered_vmux_fallback) {
            // THIS IS A HARDCODED VALUE FOR AETHER 3/15/2025
            // If the rocket has been in flight for over 80 seconds, we swap the FSM camera feed to the bulkhead camera
            has_triggered_vmux_fallback = true;
            rocket_data.command_flags.FSM_should_swap_camera_feed = true;
        }

        if (current_state == FSMState(STATE_IDLE) || current_state == FSMState(STATE_SAFE) ||
            current_state == FSMState(STATE_PYRO_TEST) || (current_time - launch_time) > 1800000) {
            TelemetryCommand command;
            if (tlm.receive(&command, 200)) {
                if (command.valid()) {
                    tlm.acknowledgeReceived();
                    handle_tlm_command(command, current_state);
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
template<typename Hw>
ErrorCode RocketSystems<Hw>::init_systems() {
    gpioDigitalWrite(LED_ORANGE, HIGH);
    ErrorCode c = hw.init_all();
    if (c != ErrorCode::NoError) {
        return c;
    }
    INIT_SYSTEM(led);
    INIT_SYSTEM(buzzer);
    INIT_SYSTEM(b2b);
#ifdef ENABLE_TELEM
    INIT_SYSTEM(tlm);
#endif
    gpioDigitalWrite(LED_ORANGE, LOW);
    return NoError;
}
#undef INIT_SYSTEM

/**
 * @brief Initializes the systems, and then creates and starts the thread for each system.
 *        If initialization fails, then this enters an infinite loop.
 */
template<typename Hw>
[[noreturn]] void RocketSystems<Hw>::begin() {
    Serial.println("Starting Systems...");
    ErrorCode init_error_code = init_systems();
    if (init_error_code != NoError) {
        // todo some message probably
        Serial.print("Had Error: ");
        Serial.print((int) init_error_code);
        Serial.print("\n");
        Serial.flush();
        update_error_LED(init_error_code);
        while (true) {

        }
    }

    START_THREAD(orientation, SENSOR_CORE, this, 10);
    START_THREAD(logger, DATA_CORE, this, 15);
    START_THREAD(accelerometers, SENSOR_CORE, this, 13);
    START_THREAD(barometer, SENSOR_CORE, this, 12);
    START_THREAD(gps, SENSOR_CORE, this, 8);
    START_THREAD(voltage, SENSOR_CORE, this, 9);
    START_THREAD(pyro, SENSOR_CORE, this, 14);
    START_THREAD(magnetometer, SENSOR_CORE, this, 11);
    START_THREAD(cam, SENSOR_CORE, this, 16);
    START_THREAD(kalman, SENSOR_CORE, this, 7);
    START_THREAD(fsm, SENSOR_CORE, this, 8);
    START_THREAD(buzzer, SENSOR_CORE, this, 6);
#ifdef ENABLE_TELEM
    START_THREAD(telemetry, SENSOR_CORE, this, 15);
#endif

    buzzer.play_tune(free_bird, FREE_BIRD_LENGTH);

    while (true) {
        THREAD_SLEEP(1000);
        Serial.print("Running (Log Latency: ");
        Serial.print(rocket_data.log_latency.getLatency());
        Serial.println(")");
    }
}
