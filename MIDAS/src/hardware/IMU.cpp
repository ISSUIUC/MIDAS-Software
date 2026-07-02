#include "lsm6dsv320x.h"

#include "util/errors.h"
#include "sensors.h" 

#define NUM_DIRECTIONS 3
#define NUM_READINGS_FOR_CALIB 50

/**
 * @brief Singleton interface to the LSM6DSV320X inertial measurement unit.
 *
 * Communicates with the IMU over SPI using the configured chip-select
 * and interrupt pins.
 */
LSM6DSV320XClass LSM6DSV(SPI, IMU_CS_PIN, IMU_IRQ_PIN);

/**
 * @brief Reads the latest measurements from the IMU.
 *
 * Retrieves any newly available low-G acceleration, high-G acceleration,
 * and angular velocity measurements based on the sensor's status register.
 * Measurements whose data-ready flags are not asserted retain their
 * default-initialized values.
 *
 * @return IMU data packet containing the available sensor measurements.
 */
IMU IMUSensor::read() {

    // Determine which sensor outputs have new data available.
    lsm6dsv320x_status_reg_t status = LSM6DSV.get_status();

    IMU reading{};

    // Read the low-G accelerometer if fresh data is available.
    if(status.xlda){
        LSM6DSV.get_lowg_acceleration_from_fs8_to_g(&reading.lowg_acceleration.ax,
                                                    &reading.lowg_acceleration.ay,
                                                    &reading.lowg_acceleration.az);
    }

    // Read the high-G accelerometer if fresh data is available.
    if(status.xlhgda){
        LSM6DSV.get_highg_acceleration_from_fs64_to_g(&reading.highg_acceleration.ax,
                                                      &reading.highg_acceleration.ay,
                                                      &reading.highg_acceleration.az);
    }

    // Read the gyroscope if fresh data is available.
    if(status.gda){
        LSM6DSV.get_angular_velocity_from_fs2000_to_dps(&reading.angular_velocity.vx,
                                                        &reading.angular_velocity.vy,
                                                        &reading.angular_velocity.vz);
    }

    return reading;
}

/**
 * @brief Reads the Sensor Fusion Low Power (SFLP) outputs.
 *
 * Retrieves the orientation quaternion, estimated gyroscope bias,
 * and gravity vector produced by the IMU's onboard sensor fusion
 * engine.
 *
 * @return IMU_SFLP packet containing the latest sensor fusion outputs.
 */
IMU_SFLP IMUSensor::read_sflp() {

    IMU_SFLP reading;

    uint16_t val[4];

    // Retrieve the estimated orientation quaternion.
    LSM6DSV.lsm6dsv320x_sflp_quaternion_raw_get(val);

    reading.quaternion.w = LSM6DSV.sflp_quaternion_raw_to_float(val[0]);
    reading.quaternion.x = LSM6DSV.sflp_quaternion_raw_to_float(val[1]);
    reading.quaternion.y = LSM6DSV.sflp_quaternion_raw_to_float(val[2]);
    reading.quaternion.z = LSM6DSV.sflp_quaternion_raw_to_float(val[3]);

    // Retrieve the estimated gyroscope bias.
    LSM6DSV.sflp_gbias_raw_get((int16_t*)&val);

    reading.gyro_bias.vx = LSM6DSV.sflp_gbias_raw_to_mdps(val[0]) / 1000.0;
    reading.gyro_bias.vy = LSM6DSV.sflp_gbias_raw_to_mdps(val[1]) / 1000.0;
    reading.gyro_bias.vz = LSM6DSV.sflp_gbias_raw_to_mdps(val[2]) / 1000.0;

    // Retrieve the estimated gravity vector.
    LSM6DSV.sflp_gravity_raw_get((int16_t*)&val);

    reading.gravity.ax = LSM6DSV.sflp_gravity_raw_to_mg(val[0]) / 1000.0;
    reading.gravity.ay = LSM6DSV.sflp_gravity_raw_to_mg(val[1]) / 1000.0;
    reading.gravity.az = LSM6DSV.sflp_gravity_raw_to_mg(val[2]) / 1000.0;

    return reading;
}

/**
 * @brief Audio cues used during the IMU calibration procedure.
 *
 * Different tone sequences indicate calibration ready, advancing to the
 * next orientation, successful completion, or calibration abort.
 */
#define XLC_TONE_PITCH Sound{3000, 65}
#define XLC_TONE_PITCH_LONG Sound{3000, 250}
#define XLC_TONE_WAIT Sound{0, 50}

Sound xl_calib_rdy[C_XL_LENGTH] = {XLC_TONE_PITCH, XLC_TONE_WAIT, XLC_TONE_PITCH};
Sound xl_calib_next_axis[C_XL_LENGTH] = {XLC_TONE_PITCH, XLC_TONE_WAIT, XLC_TONE_WAIT};
Sound xl_calib_done[C_XL_LENGTH] = {XLC_TONE_PITCH_LONG, XLC_TONE_PITCH, XLC_TONE_WAIT};
Sound xl_calib_abort[C_XL_LENGTH] = {XLC_TONE_PITCH_LONG, XLC_TONE_PITCH_LONG, XLC_TONE_PITCH_LONG};

/**
 * @brief Restores the previously saved IMU calibration from EEPROM.
 *
 * Exits calibration mode and reloads the stored high-G accelerometer
 * bias values.
 *
 * @param eeprom EEPROM controller containing the saved calibration data.
 */
void IMUSensor::restore_calibration(EEPROMController& eeprom) {
    // Exit calibration mode.
    calibration_state = IMUSensor::IMUCalibrationState::NONE;

    // Restore the saved high-G accelerometer bias.
    calibration_sensor_bias = eeprom.data.lsm6dsv320x_hg_xl_bias;
}

/**
 * @brief Aborts the current IMU calibration procedure.
 *
 * Plays the calibration abort tone and restores the previously saved
 * calibration values.
 *
 * @param buzzer Buzzer used to indicate calibration status.
 * @param eeprom EEPROM controller containing the saved calibration.
 */
void IMUSensor::abort_calibration(BuzzerController& buzzer, EEPROMController& eeprom) {
    buzzer.play_tune(xl_calib_abort, C_XL_LENGTH);
    restore_calibration(eeprom);
}

/**
 * @brief Begins the six-orientation IMU calibration procedure.
 *
 * Starts calibration in the +X orientation unless calibration is already
 * in progress.
 *
 * @param buzzer Buzzer used to guide the user through calibration.
 */
void IMUSensor::begin_calibration(BuzzerController& buzzer) {
    // Ignore the request if calibration is already active.
    if(calibration_state != IMUSensor::IMUCalibrationState::NONE) {
        return;
    }

    // Signal that calibration is ready to begin.
    buzzer.play_tune(xl_calib_rdy, C_XL_LENGTH);

    // Record the calibration start time.
    _calib_begin_timestamp = millis();

    // Begin with the +X orientation.
    calibration_state = IMUSensor::IMUCalibrationState::CALIB_PX;
}

/**
 * @brief Determines whether the IMU is correctly oriented for calibration.
 *
 * Compares the measured low-G acceleration against the expected
 * gravitational acceleration along a single axis.
 *
 * @param lowg_axis_reading Measured acceleration on the selected axis.
 * @param nominal_axis_value Expected acceleration (±1 g).
 *
 * @return True if the reading is within the acceptable tolerance.
 */
bool IMUSensor::accept_calib_reading(float lowg_axis_reading, float nominal_axis_value) {
    constexpr static float kLowgMaxDeviation = 0.012;
    return std::abs(lowg_axis_reading - nominal_axis_value) < kLowgMaxDeviation;
}

/**
 * @brief Advances calibration to the next orientation.
 *
 * Once all six orientations have been completed, the computed high-G
 * accelerometer bias is saved to EEPROM.
 *
 * @param buzzer Buzzer used to indicate calibration progress.
 * @param eeprom EEPROM controller used to store the calibration.
 */
void IMUSensor::next_calib(BuzzerController& buzzer, EEPROMController& eeprom) {
    IMUCalibrationState next_state = static_cast<IMUCalibrationState>(static_cast<int>(calibration_state) + 1);

    if (next_state == IMUCalibrationState::CALIB_DONE) {
        // Calibration is complete. Save the computed bias values.
        calibration_state = IMUCalibrationState::NONE;
        eeprom.data.lsm6dsv320x_hg_xl_bias = calibration_sensor_bias;
        eeprom.commit();
        buzzer.play_tune(xl_calib_done, C_XL_LENGTH);
    } else {
        // Advance to the next calibration orientation.
        calibration_state = next_state;
        buzzer.play_tune(xl_calib_next_axis, C_XL_LENGTH);
    }

    // Reset the sample counter for the next orientation.
    _calib_valid_readings = 0;
}

/**
 * @brief Processes a calibration sample.
 *
 * During calibration, the rocket is placed in each of the six principal
 * orientations (+X, -X, +Y, -Y, +Z, -Z). Once enough valid samples have
 * been collected for an orientation, the corresponding high-G
 * accelerometer bias is computed.
 *
 * @param lowg_reading Low-G accelerometer measurement.
 * @param highg_reading High-G accelerometer measurement.
 * @param buzzer_indicator Buzzer used for user feedback.
 * @param eeprom EEPROM controller used to save calibration results.
 */
void IMUSensor::calib_reading(Acceleration lowg_reading, Acceleration highg_reading, BuzzerController& buzzer_indicator, EEPROMController& eeprom) {

    switch (calibration_state) {

        // +X orientation.
        case IMUSensor::IMUCalibrationState::CALIB_PX:

            if(accept_calib_reading(lowg_reading.ax, 1.0)) {
                float cur_offset = 1.0 - highg_reading.ax;
                _calib_valid_readings++;
                _calib_average += cur_offset;

                // Advance once enough valid samples have been collected.
                if (_calib_valid_readings >= NUM_READINGS_FOR_CALIB) {
                    Serial.println("[+X] Good");
                    next_calib(buzzer_indicator, eeprom);
                }
            }
            break;

        // -X orientation. Compute the X-axis bias.
        case IMUSensor::IMUCalibrationState::CALIB_NX:
            if(accept_calib_reading(lowg_reading.ax, -1.0)) {
                float cur_offset = -1.0 - highg_reading.ax;
                _calib_valid_readings++;
                _calib_average += cur_offset;

                if (_calib_valid_readings >= NUM_READINGS_FOR_CALIB) {
                    float overall_offset = _calib_average / (NUM_READINGS_FOR_CALIB * 2);
                    Serial.println("[-X] Good.");
                    calibration_sensor_bias.ax = overall_offset;
                    _calib_average = 0.0;
                    next_calib(buzzer_indicator, eeprom);
                }
            }
            break;

        // +Y orientation.
        case IMUSensor::IMUCalibrationState::CALIB_PY:

            if(accept_calib_reading(lowg_reading.ay, 1.0)) {
                float cur_offset = 1.0 - highg_reading.ay;
                _calib_valid_readings++;
                _calib_average += cur_offset;

                if (_calib_valid_readings >= NUM_READINGS_FOR_CALIB) {
                    Serial.println("[+Y] Good");
                    next_calib(buzzer_indicator, eeprom);
                }
            }
            break;

        // -Y orientation. Compute the Y-axis bias.
        case IMUSensor::IMUCalibrationState::CALIB_NY:
            if(accept_calib_reading(lowg_reading.ay, -1.0)) {
                float cur_offset = -1.0 - highg_reading.ay;
                _calib_valid_readings++;
                _calib_average += cur_offset;

                if (_calib_valid_readings >= NUM_READINGS_FOR_CALIB) {
                    float overall_offset = _calib_average / (NUM_READINGS_FOR_CALIB * 2);
                    Serial.println("[-Y] Good.");
                    calibration_sensor_bias.ay = overall_offset;
                    _calib_average = 0.0;
                    next_calib(buzzer_indicator, eeprom);
                }
            }
            break;

        // +Z orientation.
        case IMUSensor::IMUCalibrationState::CALIB_PZ:

            if(accept_calib_reading(lowg_reading.az, 1.0)) {
                float cur_offset = 1.0 - highg_reading.az;
                _calib_valid_readings++;
                _calib_average += cur_offset;

                if (_calib_valid_readings >= NUM_READINGS_FOR_CALIB) {
                    Serial.println("[+Z] Good");
                    next_calib(buzzer_indicator, eeprom);
                }
            }
            break;

        // -Z orientation. Compute the Z-axis bias.
        case IMUSensor::IMUCalibrationState::CALIB_NZ:
            if(accept_calib_reading(lowg_reading.az, -1.0)) {
                float cur_offset = -1.0 - highg_reading.az;
                _calib_valid_readings++;
                _calib_average += cur_offset;

                if (_calib_valid_readings >= NUM_READINGS_FOR_CALIB) {
                    float overall_offset = _calib_average / (NUM_READINGS_FOR_CALIB * 2);
                    Serial.println("[-Z] Good.");
                    calibration_sensor_bias.az = overall_offset;
                    _calib_average = 0.0;
                    next_calib(buzzer_indicator, eeprom);
                }
            }
            break;

        // Invalid calibration state. Reset the calibration state machine.
        default:
            calibration_state = IMUCalibrationState::NONE;
            break;
    }
}

/**
 * @brief Initializes and configures the LSM6DSV320X IMU.
 *
 * Verifies communication with the sensor, resets it to a known state,
 * configures the accelerometers and gyroscope, enables the Sensor Fusion
 * Low Power (SFLP) engine, and disables the default low-pass filters.
 *
 * @return ErrorCode::NoError if initialization succeeds.
 * @return ErrorCode::IMUCouldNotBeInitialized if the IMU cannot be detected.
 */
ErrorCode IMUSensor::init() {
    uint8_t whoami;

    // Verify that the connected device matches the expected IMU.
    LSM6DSV.device_id_get(&whoami);
    if(whoami != LSM6DSV320X_ID)
        return IMUCouldNotBeInitialized;

    // Reset the IMU to its power-on state.
    LSM6DSV.sw_por();

    // Configure the low-G accelerometer, gyroscope, and high-G
    // accelerometer to operate at 480 Hz in high-performance mode.
    LSM6DSV.xl_setup(LSM6DSV320X_ODR_AT_480Hz, LSM6DSV320X_XL_HIGH_PERFORMANCE_MD);
    LSM6DSV.gy_setup(LSM6DSV320X_ODR_AT_480Hz, LSM6DSV320X_GY_HIGH_PERFORMANCE_MD);
    LSM6DSV.hg_xl_data_rate_set(LSM6DSV320X_HG_XL_ODR_AT_480Hz, 1);

    // Configure the measurement ranges.
    LSM6DSV.hg_xl_full_scale_set(LSM6DSV320X_64g);
    LSM6DSV.xl_full_scale_set(LSM6DSV320X_8g);
    LSM6DSV.gy_full_scale_set(LSM6DSV320X_2000dps);

    // Enable the onboard Sensor Fusion Low Power engine.
    LSM6DSV.sflp_enable_set(1);

    // Configure the digital filter settling behavior.
    LSM6DSV.filt_settling_mask_set(false, false, false);

    // Disable the default low-pass filters to preserve sensor bandwidth.
    LSM6DSV.filt_gy_lp1_set(PROPERTY_DISABLE);
    // lsm6dsv320x_filt_gy_lp1_bandwidth_set(&dev_ctx, lsm6dsv320x_GY_ULTRA_LIGHT);
    LSM6DSV.filt_xl_lp2_set(PROPERTY_DISABLE);

    return NoError;
}