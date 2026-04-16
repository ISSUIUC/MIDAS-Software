#include "systems.h"
#include "sensor_data.h"
#include "hal.h"
#include "gnc/ekf.h"
#include "gnc/mqekf.h"
#include "log_format_AUTOGEN.h"
#include "midas_shell_commands.h"

static StaticSemaphore_t spi_mutex_buffer;
SemaphoreHandle_t spi_mutex;

#define ENABLE_TELEM

#define METALOG_TEST

void log_parse_tree(LogSink& log, size_t node_size, size_t node_count, uint8_t* data) {
    // To recreate log formats, we need data to reconstruct the parse tree for log format
    // This means knowing the size of each node and the pre-order traversal.
    // We will store first the size of the node (each entry type), then a byte array of the pre order traversal.
    // This lets us recreate the tree by converting the byte array into an array of nodes, then recreating the preorder traversal.
    // For degenerate trees (lists), we will do the same thing, but they will be decoded as a list instead.
    log.write_meta((uint8_t*)&node_size, sizeof(size_t)); // Store the node size
    log.write_meta((uint8_t*)&node_count, sizeof(size_t)); // Store number of nodes
    log.write_meta(data, node_size * node_count); // Store raw parse tree data
}

/**
 * @brief These are all the functions that will run in each task
 * Each function has a `while (true)` loop within that should not be returned out of or yielded in any way
 *
 * The `DECLARE_THREAD` macro creates a function whose name is suffixed by _thread, and annotates it with [[noreturn]]
 */
DECLARE_THREAD(logger, RocketSystems *arg)
{
    log_begin(arg->log_sink);
    int meta_delay_ctr = 0; //  Will cause meta logs to write slower intentionally

    constexpr size_t meta_logging_header_size = (sizeof(size_t)*8) + EEPROM_SIZE + sizeof(LogFormatMetaEntry)*(LOG_META_ENTRY_COUNT+EEPROM_META_ENTRY_COUNT) + sizeof(LogDiscMapEntry)*LOG_DISCMAP_COUNT;

    // Write header size
    arg->log_sink.write_meta((uint8_t*)&meta_logging_header_size, sizeof(size_t));

    // Write meta log data + EEPROM
    // We first write the EEPROM data, prefixed by the EEPROM size.
    arg->log_sink.write_meta((uint8_t*)&EEPROM_SIZE, sizeof(size_t));
    arg->log_sink.write_meta((uint8_t*)&arg->eeprom.data, EEPROM_SIZE);

    // Log the formats themselves
    log_parse_tree(arg->log_sink, sizeof(LogFormatMetaEntry), LOG_META_ENTRY_COUNT, (uint8_t*)LOG_META_ENTRIES);
    log_parse_tree(arg->log_sink, sizeof(LogDiscMapEntry), LOG_DISCMAP_COUNT, (uint8_t*)LOG_DISCMAP_TABLE);
    log_parse_tree(arg->log_sink, sizeof(LogFormatMetaEntry), EEPROM_META_ENTRY_COUNT, (uint8_t*)EEPROM_META_ENTRIES);

    while (true) {
        log_data(arg->log_sink, arg->rocket_data);

        arg->rocket_data.log_latency.tick();
        meta_delay_ctr++;

        MetaLogging::MetaLogEntry entry;

        if (meta_delay_ctr >= 100) {
            if(arg->meta_logging.get_queued(&entry)) {
                uint8_t buf[72];
                size_t total_size = sizeof(MetaDataCode) + entry.size;
                memcpy(buf, &entry.log_type, sizeof(MetaDataCode));
                memcpy(buf + sizeof(MetaDataCode), &entry.data, entry.size);
                arg->log_sink.write_meta(buf, total_size);
            }
        }

        arg->rocket_data.err_flags.log_wr_err = arg->log_sink.failed_wr;
        arg->rocket_data.err_flags.log_mr_err = arg->log_sink.failed_mr;

        THREAD_SLEEP(1);
    }
}



DECLARE_THREAD(barometer, RocketSystems* arg) {
    // Reject single rogue barometer readings that are very different from the immediately prior reading
    // Will only reject a certain number of readings in a row
    Barometer prev_reading;
    constexpr float altChgThreshold = 200;  // meters
    constexpr float presChgThreshold = 500; // milibars
    constexpr float tempChgThreshold = 10;  // degrees C
    constexpr unsigned int maxConsecutiveRejects = 3;
    unsigned int rejects = maxConsecutiveRejects; // Always accept first reading
    while (true)
    {
        // xSemaphoreTake(spi_mutex, portMAX_DELAY);
        Barometer reading = arg->sensors.barometer.read();
        // xSemaphoreGive(spi_mutex);
        bool is_rogue = std::abs(prev_reading.altitude - reading.altitude) > altChgThreshold;
        // std::abs(prev_reading.pressure - reading.pressure) > presChgThreshold ||
        // std::abs(prev_reading.temperature - reading.temperature) > tempChgThreshold;
        // TODO: Log when we receive a rejection!
        if (is_rogue && rejects++ < maxConsecutiveRejects)
            arg->rocket_data.barometer.update(prev_reading); // Reuse old reading, reject new reading
        else
        {
            rejects = 0;
            arg->rocket_data.barometer.update(reading);
            prev_reading = reading; // Only update prev_reading with accepted readings
        }

        THREAD_SLEEP(6);
    }
}

DECLARE_THREAD(imuthread, RocketSystems *arg)
{ // This needs edits

    arg->sensors.imu.restore_calibration(arg->eeprom);
    float max_accel = 0;
    bool has_logged = false;
    double max_accel_time = 0;

    while (true)
    {
        xSemaphoreTake(spi_mutex, portMAX_DELAY);
        IMU imudata = arg->sensors.imu.read();
        IMU_SFLP sflp = arg->sensors.imu.read_sflp();
        xSemaphoreGive(spi_mutex);

        // Sensor calibration, if it is triggered.
        if(arg->sensors.imu.calibration_state != IMUSensor::IMUCalibrationState::NONE) {
            arg->sensors.imu.calib_reading(imudata.lowg_acceleration, imudata.highg_acceleration, arg->buzzer, arg->eeprom);

            if(arg->sensors.imu.get_time_since_calibration_start() > 60000) {
                // Abort calibration if it isn't finished in 60s.
                arg->sensors.imu.abort_calibration(arg->buzzer, arg->eeprom);
            }
        }

        // Sensor biases
        Acceleration bias = arg->sensors.imu.calibration_sensor_bias;

        float prev_accel = arg->rocket_data.imu.getRecentUnsync().highg_acceleration.ax;
        

        imudata.highg_acceleration.ax = imudata.highg_acceleration.ax + bias.ax;
        imudata.highg_acceleration.ay = imudata.highg_acceleration.ay + bias.ay;
        imudata.highg_acceleration.az = imudata.highg_acceleration.az + bias.az;

        if(arg->rocket_data.fsm_state.getRecentUnsync().state == FSMState::STATE_BOOST) { // max accel can be during second boost, 
            // but once the new fsm is implemented this will be changed to just "boost" and work (i think) - mihir
            if(prev_accel < imudata.highg_acceleration.ax) {
                max_accel = imudata.highg_acceleration.ax;
                max_accel_time = pdTICKS_TO_MS(xTaskGetTickCount());
            }
        }

        if(!has_logged) {
            if(arg->rocket_data.fsm_state.getRecentUnsync().state == FSMState::STATE_LANDED) {
                arg->meta_logging.log_data(MetaDataCode::DATA_MAX_ACCEL, max_accel);
                arg->meta_logging.log_event(MetaDataCode::EVENT_TMAX_ACCEL, max_accel_time);
                has_logged = true;
            }
        }

        arg->rocket_data.imu.update(imudata);
        arg->rocket_data.sflp.update(sflp);
        
        THREAD_SLEEP(5);
    }
}

DECLARE_THREAD(magnetometer, RocketSystems* arg) {
    arg->sensors.magnetometer.restore_calibration(arg->eeprom);
    int i = 0;
    while (true) {
        // xSemaphoreTake(spi_mutex, portMAX_DELAY);
        Magnetometer reading = arg->sensors.magnetometer.read();
        // xSemaphoreGive(spi_mutex);

        // Sensor calibration
        if(arg->sensors.magnetometer.in_calibration_mode) {
            arg->sensors.magnetometer.calib_reading(reading, arg->eeprom, arg->buzzer);
            // Mag calibration handles its own calibration timing.
        }

        // Sensor biases
        Magnetometer b = arg->sensors.magnetometer.calibration_bias_hardiron; // "Hard iron" / origin offset.
        Magnetometer s = arg->sensors.magnetometer.calibration_bias_softiron; // "Soft iron" / scale offset.
        reading.mx = (reading.mx - b.mx) / s.mx;
        reading.my = (reading.my - b.my) / s.my;
        reading.mz = (reading.mz - b.mz) / s.mz;

        arg->rocket_data.magnetometer.update(reading);

        THREAD_SLEEP(50);
    }
}

DECLARE_THREAD(gps, RocketSystems *arg)
{
    while (true)
    {
        if (arg->sensors.gps.valid())
        {
            GPS reading = arg->sensors.gps.read();
            arg->rocket_data.gps.update(reading);
        }
        // GPS waits internally
        THREAD_SLEEP(1);
    }
}

DECLARE_THREAD(pyro, RocketSystems *arg)
{

    while (true)
    {
        FSMData current_fsm = arg->rocket_data.fsm_state.getRecentUnsync();
        AngularKalmanData akf_data = arg->rocket_data.angular_kalman_data.getRecentUnsync();
        KalmanData ekf_data = arg->rocket_data.kalman.getRecentUnsync();
        CommandFlags &command_flags = arg->rocket_data.command_flags;
        double current_time = pdTICKS_TO_MS(xTaskGetTickCount());
        double launch_time = arg->fsm.get_launch_time();

        double time_since_launch = (current_time - launch_time);
        const FSMConfiguration& fsm_cfg = arg->fsm.get_cfg();

        PyroTickData tick_data = {
            current_fsm,
            akf_data,
            ekf_data,
            fsm_cfg,
            command_flags,
            current_time,
            time_since_launch
        };

        PyroState new_pyro_state = arg->sensors.pyro.tick(tick_data);

        // Actually update the pyro state!
        gpioDigitalWrite(PYRO_GLOBAL_ARM_PIN, new_pyro_state.is_global_armed ? HIGH : LOW);
        for(int i = 0; i < MIDAS_NUM_PYROS; i++) {
            gpioDigitalWrite(PYRO_PINS[i], new_pyro_state.channel_firing[i] ? HIGH : LOW);
        }

        arg->rocket_data.pyro.update(new_pyro_state);
        arg->led.update();

        THREAD_SLEEP(10);
    }
}

DECLARE_THREAD(voltage, RocketSystems* arg) {
    while (true) {
        Voltage reading2 = arg->sensors.voltage.read();
        arg->rocket_data.voltage.update(reading2);

        THREAD_SLEEP(100);
    }
}

//run threads

void fsm_transitioned_to(FSMState& new_state, FSMState& old_state, RocketSystems* sys, double current_time) {
    // Do something, NO delays allowed!
    AngularKalmanData cur_orientation = sys->rocket_data.angular_kalman_data.getRecentUnsync();
    switch (new_state) {
        case FSMState::STATE_BOOST:
            sys->meta_logging.log_event(MetaDataCode::EVENT_TLAUNCH, current_time);
            sys->meta_logging.log_data(MetaDataCode::DATA_LAUNCHSITE_BARO, sys->rocket_data.barometer.getRecentUnsync());
            sys->meta_logging.log_data(MetaDataCode::DATA_LAUNCHSITE_GPS, sys->rocket_data.gps.getRecentUnsync());
            sys->meta_logging.log_data(MetaDataCode::DATA_LAUNCH_INITIAL_TILT, cur_orientation.sflp_tilt);
            break;
        case FSMState::STATE_COAST:
            sys->meta_logging.log_event(MetaDataCode::EVENT_TBURNOUT, current_time);
            sys->meta_logging.log_data(MetaDataCode::DATA_TILT_AT_BURNOUT, cur_orientation.sflp_tilt);
            sys->meta_logging.log_data(MetaDataCode::DATA_ALT_AT_BURNOUT, sys->rocket_data.barometer.getRecentUnsync().altitude);
            break;
        // case FSMState::STATE_SECOND_BOOST:
        //     sys->meta_logging.log_event(MetaDataCode::EVENT_TIGNITION, current_time);
        //     sys->meta_logging.log_data(MetaDataCode::DATA_TILT_AT_IGNITION, cur_orientation.sflp_tilt);
        //     break;
        case FSMState::STATE_DROGUE:
            sys->meta_logging.log_event(MetaDataCode::EVENT_TAPOGEE, current_time);
            break;
        case FSMState::STATE_MAIN:
            sys->meta_logging.log_event(MetaDataCode::EVENT_TMAIN, current_time);
            break;
        default:
            break;
    }
}

// This thread has a bit of extra logic since it needs to play a tune exactly once the sustainer ignites
DECLARE_THREAD(fsm, RocketSystems *arg)
{
    FSM& fsm = arg->fsm;

    // Read in data from EEPROM
    FSMConfiguration cfg = arg->eeprom.data.fsm_config;
    if(!fsm.set_cfg(cfg)) {
        fsm.set_crc_fail();
        arg->rocket_data.err_flags.fsm_crc_err = true;
    }

    m_shell_load_fsm_config(cfg);

    bool already_played_freebird = false;
    double last_time_led_flash = pdTICKS_TO_MS(xTaskGetTickCount());
    
    float max_descent_rate = 0;
    bool has_logged = false;
    double max_descent_rate_time = 0;
    
    double last_time_err_flash = pdTICKS_TO_MS(xTaskGetTickCount());
    arg->rocket_data.fsm_state.update(FSMData{FSMState::STATE_SAFE, 0});
    while (true)
    {
        FSMData current_state_data = arg->rocket_data.fsm_state.getRecentUnsync();
        StateEstimate state_estimate(arg->rocket_data);
        CommandFlags &telemetry_commands = arg->rocket_data.command_flags;
        KalmanData kfd = arg->rocket_data.kalman.getRecentUnsync();
        double current_time = pdTICKS_TO_MS(xTaskGetTickCount());
        const FSMConfiguration& fsm_cfg = arg->fsm.get_cfg();

        FSMState current_state = current_state_data.state;

        FSMTickData tick_data = {current_state_data, telemetry_commands, state_estimate, kfd, fsm_cfg, current_time};

        FSMData next_state = fsm.tick_fsm(tick_data);

        arg->rocket_data.fsm_state.update(next_state);
        
        if(current_state != next_state.state) {
            fsm_transitioned_to(next_state.state, current_state, arg, current_time);       
        }

        if(arg->rocket_data.err_flags.encode() != 0) {
            if ((current_time - last_time_err_flash) > 100)
            {
                // Fast flash red LED if any midas err, including crc err!
                last_time_err_flash = current_time;
                arg->led.toggle(LED::RED);
            }
        }

        if (current_state == FSMState::STATE_SAFE)
        {

            if ((current_time - last_time_led_flash) > 250)
            {
                // Flashes green LED at 4Hz while in SAFE mode.
                last_time_led_flash = current_time;
                arg->led.toggle(LED::GREEN);
            }
        }
        else
        {
            arg->led.set(LED::GREEN, LOW);
        }

        if ((current_state == FSMState::STATE_PYRO_TEST) && !arg->buzzer.is_playing())
        {
            arg->buzzer.play_tune(warn_tone, WARN_TONE_LENGTH);
        }

        if (current_state == FSMState::STATE_LANDED && !arg->buzzer.is_playing())
        {
            arg->buzzer.play_tune(land_tone, LAND_TONE_LENGTH);
        }

        if (current_state == FSMState::STATE_BOOST && next_state.current_motor == 2 && !already_played_freebird)
        {
            arg->buzzer.play_tune(free_bird, FREE_BIRD_LENGTH);
            already_played_freebird = true;
        }

        // FSM-based camera control
        if (arg->rocket_data.command_flags.FSM_should_set_cam_feed_cam1)
        {
            // Swap camera feed to MUX 1 (Side-facing camera) at launch.
            arg->rocket_data.command_flags.FSM_should_set_cam_feed_cam1 = false;
            arg->b2b.camera.vmux_set(SIDE_CAMERA);

            arg->b2b.camera.camera_on(SIDE_CAMERA);
            arg->b2b.camera.camera_on(BULKHEAD_CAMERA);
        }

        if (arg->rocket_data.command_flags.FSM_should_power_save)
        {
            arg->rocket_data.command_flags.FSM_should_power_save = false;

            arg->b2b.camera.vtx_off();
        }

        if (arg->rocket_data.command_flags.FSM_should_swap_camera_feed)
        {
            // Swap camera feed to MUX 2 (recovery bay camera)
            arg->rocket_data.command_flags.FSM_should_swap_camera_feed = false;
            arg->b2b.camera.vmux_set(BULKHEAD_CAMERA);
        }
        //MAX DESCENT RATE METADATA LOGGING - incomplete
        if(arg->rocket_data.fsm_state.getRecentUnsync().state >= FSMState::STATE_DROGUE && arg->rocket_data.fsm_state.getRecentUnsync().state < FSMState::STATE_LANDED) {
            if(max_descent_rate < state_estimate.vertical_speed) {
                max_descent_rate = state_estimate.vertical_speed;
                max_descent_rate_time = pdTICKS_TO_MS(xTaskGetTickCount());
            }
        }

        if(!has_logged) {
            if(arg->rocket_data.fsm_state.getRecentUnsync().state == FSMState::STATE_LANDED) {
                arg->meta_logging.log_data(MetaDataCode::DATA_MAX_DESCENT_RATE, max_descent_rate);
                arg->meta_logging.log_event(MetaDataCode::EVENT_TMAX_DESCENT_RATE, max_descent_rate_time);
                has_logged = true;
            }
        }
        THREAD_SLEEP(50);
    }
}

DECLARE_THREAD(buzzer, RocketSystems *arg)
{
    double last_beep_beep = pdMS_TO_TICKS(xTaskGetTickCount());
    while (true)
    {
        double cur_time = pdMS_TO_TICKS(xTaskGetTickCount());
        if(cur_time - last_beep_beep > 8000 && arg->rocket_data.fsm_state.getRecentUnsync().state == FSMState::STATE_ARMED) {
            // Get cont and fsm failure data
            bool cont[4] = {false, false, false, false};
            bool fsm_fail = arg->fsm.get_cfg().crc32 == FSM_CRC_FAIL_STATE;

            Voltage v = arg->rocket_data.voltage.getRecentUnsync();
            for (int i = 0; i < 4; i++) {
                cont[i] = v.continuity[i] > 3.0;
            }

            arg->buzzer.report_beeps(cont, fsm_fail);
            last_beep_beep = cur_time;
        }

        arg->buzzer.tick();
        THREAD_SLEEP(10);
    }
}

// angularkalmandata needs updates
DECLARE_THREAD(angularkalman, RocketSystems *arg)
{ //
    mqekf.initialize(arg);
    // Serial.println("Initialized mqekf :(");
    TickType_t last = xTaskGetTickCount();

    while (true)
    {
        FSMState FSM_state = arg->rocket_data.fsm_state.getRecent().state;

        if (arg->rocket_data.command_flags.should_reset_kf)
        {
            mqekf.initialize(arg);
            TickType_t last = xTaskGetTickCount();
            arg->rocket_data.command_flags.should_reset_kf = false;
        }
        IMU_SFLP current_imu_sflp = arg->rocket_data.sflp.getRecent();
        IMU current_imu = arg->rocket_data.imu.getRecent();
        Acceleration current_high_g = current_imu.highg_acceleration;
        Acceleration current_low_g = current_imu.lowg_acceleration;
        Magnetometer current_mag = arg->rocket_data.magnetometer.getRecent();
        Velocity current_angular_velocity = current_imu.angular_velocity; // degrees
        Velocity current_gyro_bias = current_imu_sflp.gyro_bias;
        AngularKalmanData current_angular_kalman = arg->rocket_data.angular_kalman_data.getRecent();

        Acceleration current_accelerations = {
            .ax = current_high_g.ax,
            .ay = current_high_g.ay,
            .az = current_high_g.az};

        float dt = pdTICKS_TO_MS(xTaskGetTickCount() - last) / 1000.0f;
        float timestamp = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000.0f;

        // Check with Divij
        mqekf.tick(dt, current_mag, current_angular_velocity, current_accelerations, FSM_state, current_gyro_bias);
        mqekf.calculate_tilt(current_imu_sflp);    
        AngularKalmanData current_state = mqekf.getState();

        arg->rocket_data.angular_kalman_data.update(current_state);

        last = xTaskGetTickCount();

        THREAD_SLEEP(50);
    }
}

DECLARE_THREAD(kalman, RocketSystems *arg)
{
    ekf.initialize(arg);
    // Serial.println("Initialized ekf :(");
    TickType_t last = xTaskGetTickCount();

    
    float max_vel = 0;
    bool has_logged = false;
    double max_vel_time = 0;


    while (true)
    {
        if (arg->rocket_data.command_flags.should_reset_kf)
        {
            ekf.initialize(arg);
            TickType_t last = xTaskGetTickCount();
            arg->rocket_data.command_flags.should_reset_kf = false;
        }

        Barometer current_barom_buf = arg->rocket_data.barometer.getRecent();

        IMU current_imu = arg->rocket_data.imu.getRecent();
        Acceleration current_high_g = current_imu.highg_acceleration;
        Acceleration current_low_g = current_imu.lowg_acceleration;

        AngularKalmanData current_angular_kalman = arg->rocket_data.angular_kalman_data.getRecent();

        FSMState FSM_state = arg->rocket_data.fsm_state.getRecent().state;
        GPS current_gps = arg->rocket_data.gps.getRecent();

        Acceleration current_accelerations = {
            .ax = current_high_g.ax,
            .ay = current_high_g.ay,
            .az = current_high_g.az}; //

        float dt = pdTICKS_TO_MS(xTaskGetTickCount() - last) / 1000.0f;
        float timestamp = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000.0f;

        // Check with Divij
        ekf.tick(dt, 13.0, current_barom_buf, current_accelerations, current_angular_kalman, FSM_state, current_gps);

        KalmanData current_state = ekf.getState();

        arg->rocket_data.kalman.update(current_state);

        last = xTaskGetTickCount();

        //float prev_vel = current_state.velocity.vx;
        if(arg->rocket_data.fsm_state.getRecentUnsync().state >= FSMState::STATE_BOOST && arg->rocket_data.fsm_state.getRecentUnsync().state < FSMState::STATE_MAIN) {
            if(max_vel < current_state.velocity.vx) {
                max_vel = current_state.velocity.vx;
                max_vel_time = pdTICKS_TO_MS(xTaskGetTickCount());
            }
        }

        if(!has_logged) {
            if(arg->rocket_data.fsm_state.getRecentUnsync().state == FSMState::STATE_LANDED) {
                arg->meta_logging.log_data(MetaDataCode::DATA_MAX_VEL, max_vel);
                arg->meta_logging.log_event(MetaDataCode::EVENT_TMAX_ACCEL, max_vel_time);
                has_logged = true;
            }
        }
        THREAD_SLEEP(50);
    }
}

void handle_tlm_command(TelemetryCommand &command, RocketSystems *arg, FSMState current_state)
{
    // maybe we should move this somewhere else but it can stay here for now
    switch (command.command)
    {
    case CommandType::RESET_KF:
        arg->rocket_data.command_flags.should_reset_kf = true;
        break;
    case CommandType::SWITCH_TO_SAFE:
        arg->rocket_data.command_flags.should_transition_safe = true;
        break;
    case CommandType::SWITCH_TO_PYRO_TEST:
        arg->rocket_data.command_flags.should_transition_pyro_test = true;
        Serial.println("Changing to pyro test");
        break;
    case CommandType::SWITCH_TO_ARMED:
        arg->rocket_data.command_flags.should_transition_armed = true;
        break;
    case CommandType::FIRE_PYRO_A:
        if (current_state == FSMState::STATE_PYRO_TEST)
        {
            arg->rocket_data.command_flags.should_fire_pyro_a = true;
        }
        break;
    case CommandType::FIRE_PYRO_B:
        if (current_state == FSMState::STATE_PYRO_TEST)
        {
            arg->rocket_data.command_flags.should_fire_pyro_b = true;
        }
        break;
    case CommandType::FIRE_PYRO_C:
        if (current_state == FSMState::STATE_PYRO_TEST)
        {
            arg->rocket_data.command_flags.should_fire_pyro_c = true;
        }
        break;
    case CommandType::FIRE_PYRO_D:
        if (current_state == FSMState::STATE_PYRO_TEST)
        {
            arg->rocket_data.command_flags.should_fire_pyro_d = true;
        }
        break;
    case CommandType::CAM_ON:
        arg->b2b.camera.vtx_on(); // comes first to avoid brownouts due to high inrush current
        arg->b2b.camera.camera_on(CAM_1);
        arg->b2b.camera.camera_on(CAM_2);
        break;
    case CommandType::CAM_OFF:
        arg->b2b.camera.vtx_off();
        arg->b2b.camera.camera_off(CAM_1);
        arg->b2b.camera.camera_off(CAM_2);
        break;
    case CommandType::TOGGLE_CAM_VMUX:
        arg->b2b.camera.vmux_toggle();
        break;
    case CommandType::CALIB_ACCEL:
        arg->sensors.imu.begin_calibration(arg->buzzer);
        break;
    case CommandType::CALIB_MAG:
        arg->sensors.magnetometer.begin_calibration(arg->buzzer);
        break;
    default:
        break; // how
    }
}

DECLARE_THREAD(cam, RocketSystems *arg)
{
    while (true)
    {

        // Check the status of the B2B chip, if it's bad then we don't waste time with an I2C check.
        if(digitalRead(B2B_READY) == LOW) {
            THREAD_SLEEP(200);
            continue;
        }

        // Check if CAM specifically is on the I2C bus
        Wire.beginTransmission(0x69);
        byte error = Wire.endTransmission();

        if (error == 0)
        {
            arg->rocket_data.cam_data.update(arg->b2b.camera.read());
        }
        else
        {
            // If failed:
            CameraData new_cam_data = arg->rocket_data.cam_data.getRecent();
            new_cam_data.camera_state = 255; // all 1s, invalid state
            arg->rocket_data.cam_data.update(new_cam_data);
            THREAD_SLEEP(1800);
        }

        THREAD_SLEEP(200);
    }
}

DECLARE_THREAD(shell, RocketSystems *arg)
{
    char line_buf[MShell::max_line_len];
    char tmp_buf[MShell::max_line_len];
    uint8_t d_read = 0;

    while(true) {
        // Wait for STATE_SAFE before activating shell
        while(arg->rocket_data.fsm_state.getRecentUnsync().state != FSMState::STATE_SAFE) {
            THREAD_SLEEP(1000);
        }

        if(arg->shell->settings.echo) {
            Serial.print("> ");
            Serial.flush();
        }

        while (arg->rocket_data.fsm_state.getRecentUnsync().state == FSMState::STATE_SAFE)
        {
            int num_bytes_avail = Serial.available();

            if(num_bytes_avail > 0) {
                Serial.read(tmp_buf, num_bytes_avail);
                for(int i = 0; i < num_bytes_avail; i++) {

                    if(tmp_buf[i] == '\r') {
                        continue;
                    }

                    if(tmp_buf[i] == '\b' || tmp_buf[i] == 0x7F) {
                        if(d_read > 0) {
                            d_read--;
                            if(arg->shell->settings.echo) {
                                Serial.print("\b \b"); // erase character on terminal
                            }
                        }
                        continue;
                    }

                    if(arg->shell->settings.echo) {
                        Serial.print(tmp_buf[i]);
                    }

                    if(tmp_buf[i] == '\n') {
                        // Process the line instead of adding it!
                        line_buf[d_read] = '\0'; // Null terminate command
                        d_read = 0;

                        MCommandExecutionResult c_res = arg->shell->execute_line(line_buf, arg);
                        Serial.print("<done> ");
                        Serial.println(static_cast<int>(c_res));

                        if(arg->shell->settings.echo) {
                            Serial.print("> ");
                            Serial.flush();
                        }

                        continue;
                    }

                    if(d_read >= MShell::max_line_len - 1) {
                        continue;
                    }

                    // Otherwise add to the buf
                    line_buf[d_read++] = tmp_buf[i];
                }
                Serial.flush();
            }

            // do stuff here

            THREAD_SLEEP(10);
        }
    }
}

DECLARE_THREAD(telemetry, RocketSystems *arg)
{
    double launch_time = 0;
    bool has_triggered_vmux_fallback = false;

    // Restore frequency from EEPROM
    // maybe have a check to make sure frequency value is in the 420-450 MHz range?
    arg->tlm.setFrequency(arg->eeprom.data.frequency);
    while (true)
    {

        arg->tlm.transmit(arg->rocket_data, arg->eeprom.data, arg->led);

        FSMState current_state = arg->rocket_data.fsm_state.getRecentUnsync().state;
        double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

        // This applies to STATE_SAFE, STATE_PYRO_TEST, and STATE_ARMED.
        if (current_state <= FSMState::STATE_ARMED)
        {
            launch_time = current_time;
            has_triggered_vmux_fallback = false;
        }

        if ((current_time - launch_time) > 79200 && !has_triggered_vmux_fallback)
        {
            // THIS IS A HARDCODED VALUE FOR AETHER II 6/21/2025 -- Value is optimal TTA from SDA
            // If the rocket has been in flight for over 79.2 seconds, we swap the FSM camera feed to the bulkhead camera
            // This is a fallback in case we can't detect the APOGEE event, so it is more conservative.
            has_triggered_vmux_fallback = true;
            arg->rocket_data.command_flags.FSM_should_swap_camera_feed = true;
        }

        if (current_state == FSMState::STATE_ARMED || current_state == FSMState::STATE_SAFE || current_state == FSMState::STATE_PYRO_TEST || (current_time - launch_time) > 1800000)
        {
            TelemetryCommand command;
            if (arg->tlm.receive(&command, 200))
            {
                if (command.valid())
                {
                    arg->tlm.acknowledgeReceived();
                    handle_tlm_command(command, arg, current_state);
                }
            }
        }
        THREAD_SLEEP(1);
    }
}

#define INIT_SYSTEM(s)               \
    do                               \
    {                                \
        ErrorCode code = (s).init(); \
        if (code != NoError)         \
        {                            \
            return code;             \
        }                            \
    } while (0)

/**
 * @brief Initializes all systems in order, returning early if a system's initialization process errors out.
 *        Turns on the Orange LED while initialization is running.
 */
ErrorCode init_systems(RocketSystems& systems) {
    digitalWrite(LED_ORANGE, HIGH);
    INIT_SYSTEM(systems.eeprom);

    // To init log sink fast, we'll set its filenumber before initing it.
    systems.log_sink.current_file_no = systems.eeprom.data.sd_file_num_last;

    INIT_SYSTEM(systems.log_sink);

    // Then, after initing we will read back the filenumber to EEPROM.
    systems.eeprom.data.sd_file_num_last = systems.log_sink.current_file_no;

    INIT_SYSTEM(systems.sensors.imu);
    INIT_SYSTEM(systems.sensors.barometer);
    INIT_SYSTEM(systems.sensors.magnetometer);
    INIT_SYSTEM(systems.sensors.voltage);
    INIT_SYSTEM(systems.sensors.pyro);
    INIT_SYSTEM(systems.led);
    INIT_SYSTEM(systems.buzzer);
    INIT_SYSTEM(systems.b2b);
#ifdef ENABLE_TELEM
    INIT_SYSTEM(systems.tlm);
#endif
    INIT_SYSTEM(systems.sensors.gps);

    // Update eeprom
    systems.eeprom.commit();

    m_shell_setup(); // Set up the MIDAS shell
    systems.shell = &m_shell_inst;
    m_shell_init_commands(systems.shell);

    digitalWrite(LED_ORANGE, LOW);
    return NoError;
}
#undef INIT_SYSTEM

/**
 * @brief Initializes the systems, and then creates and starts the thread for each system.
 *        If initialization fails, then this enters an infinite loop.
 */
[[noreturn]] void begin_systems(RocketSystems *config)
{
    Serial.println("Starting Systems...");
    spi_mutex = xSemaphoreCreateMutexStatic(&spi_mutex_buffer);


    ErrorCode init_error_code = init_systems(*config);
    if (init_error_code != NoError)
    {
        // todo some message probably
        Serial.print("Had Error: ");
        Serial.print((int)init_error_code);
        Serial.print("\n");
        Serial.flush();
        update_error_LED(init_error_code);
        while (true)
        {
        }
    }

    START_THREAD(logger, DATA_CORE, config, 15);
    START_THREAD(imuthread, SENSOR_CORE, config, 13);
    START_THREAD(barometer, SENSOR_CORE, config, 12);
    START_THREAD(gps, SENSOR_CORE, config, 8);
    START_THREAD(voltage, SENSOR_CORE, config, 9);
    START_THREAD(pyro, SENSOR_CORE, config, 14);
    START_THREAD(magnetometer, SENSOR_CORE, config, 11);
    START_THREAD(cam, SENSOR_CORE, config, 16);
    START_THREAD(kalman, SENSOR_CORE, config, 7);
    START_THREAD(fsm, SENSOR_CORE, config, 8);
    START_THREAD(buzzer, SENSOR_CORE, config, 6);
    START_THREAD(angularkalman, SENSOR_CORE, config, 7);
    START_THREAD(shell, DATA_CORE, config, 5);

    #ifdef ENABLE_TELEM
    START_THREAD(telemetry, SENSOR_CORE, config, 15);
#endif
    // play James Bond theme for MIDAS 007
    if (config->eeprom.data.serial == 007) {
        config->buzzer.play_tune(james_bond, JAMES_BOND_LENGTH);
    } else {
        config->buzzer.play_tune(free_bird, FREE_BIRD_LENGTH);
    }

    while (true)
    {
        THREAD_SLEEP(1000);
    }
}

// void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char* pcTaskName){
//     Serial.println("OVERFLOW");
//     Serial.println((char*)pcTaskName);
// }
