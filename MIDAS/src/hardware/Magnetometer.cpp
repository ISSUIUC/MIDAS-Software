#include <SparkFun_MMC5983MA_Arduino_Library.h>

#include "sensors.h"
#include "util/hal.h"

/**
 * @brief Singleton interface to the MMC5983MA magnetometer.
 */
SFE_MMC5983MA MMC5983;

/**
 * @brief Audio cues used during magnetometer calibration.
 *
 * These tone sequences indicate calibration start, user prompts during
 * calibration, successful completion, or calibration failure.
 */
#define MGC_TONE_PITCH_H Sound{3000, 50}
#define MGC_TONE_PITCH_L Sound{2200, 50}
#define MGC_TONE_PITCH_LONG Sound{3000, 250}
#define MGC_TONE_PITCH_LONG_L Sound{2000, 250}
#define MGC_TONE_WAIT Sound{0, 50}
#define MGC_TONE_NOOP Sound{0, 1}

Sound mg_calib_rdy[C_MG_LENGTH]  = {MGC_TONE_PITCH_H, MGC_TONE_WAIT, MGC_TONE_PITCH_H, MGC_TONE_WAIT, MGC_TONE_PITCH_H};
Sound mg_calib_done[C_MG_LENGTH] = {MGC_TONE_PITCH_LONG, MGC_TONE_WAIT, MGC_TONE_PITCH_H, MGC_TONE_WAIT, MGC_TONE_PITCH_H};
Sound mg_calib_inp[C_MG_LENGTH]  = {MGC_TONE_PITCH_H, MGC_TONE_NOOP, MGC_TONE_NOOP, MGC_TONE_NOOP, MGC_TONE_NOOP};
Sound mg_calib_bad[C_MG_LENGTH]  = {MGC_TONE_PITCH_LONG_L, MGC_TONE_WAIT, MGC_TONE_PITCH_LONG_L, MGC_TONE_NOOP, MGC_TONE_NOOP};

/**
 * @brief Initializes the MMC5983MA magnetometer.
 *
 * Verifies communication with the sensor over SPI.
 *
 * @return ErrorCode::NoError if initialization succeeds.
 * @return ErrorCode::MagnetometerCouldNotBeInitialized if the sensor
 *         cannot be detected.
 */
ErrorCode MagnetometerSensor::init() {
    // Verify communication with the magnetometer.
    if (!MMC5983.begin(MMC5983_CS)) {
        return ErrorCode::MagnetometerCouldNotBeInitialized;
    }

    return ErrorCode::NoError;
}

/**
 * @brief Reads the latest magnetic field measurement.
 *
 * Converts the raw 18-bit sensor output into magnetic field values
 * expressed in Gauss and applies the board coordinate transformation.
 *
 * @return Magnetometer measurement.
 */
Magnetometer MagnetometerSensor::read() {

    // Raw 18-bit sensor measurements.
    uint32_t cx, cy, cz;
    double X, Y, Z;

    MMC5983.getMeasurementXYZ(&cx, &cy, &cz);

    // Convert the unsigned raw values into normalized values centered
    // about zero.
    double sf = (double)(1 << 17);
    X = ((double)cx - sf) / sf;
    Y = ((double)cy - sf) / sf;
    Z = ((double)cz - sf) / sf;

    // Convert the normalized values to Gauss and rotate into the
    // flight computer coordinate frame.
    Magnetometer reading{Y * 8, -X * 8, -Z * 8};

    return reading;
}

/**
 * @brief Begins the magnetometer calibration procedure.
 *
 * Initializes the calibration state, resets accumulated statistics,
 * and prompts the user to begin rotating the rocket.
 *
 * @param buzzer Buzzer used to provide user feedback.
 */
void MagnetometerSensor::begin_calibration(BuzzerController& buzzer) {

    Serial.println("[MAG] Calibration begin");

    buzzer.play_tune(mg_calib_rdy, C_MG_LENGTH);

    in_calibration_mode = true;
    _calib_begin_timestamp = millis();

    // Reset calibration state.
    _calib_beeping = 1;
    _calib_max_axis = {-INFINITY, -INFINITY, -INFINITY};
    _calib_min_axis = { INFINITY,  INFINITY,  INFINITY};
    _calib_magnitude_sum = 0.0;
    _calib_num_datapoints = 0;
}

/**
 * @brief Processes a magnetometer calibration sample.
 *
 * Tracks the minimum and maximum measurement observed on each axis while
 * accumulating the average magnetic field magnitude. Periodic audio
 * prompts are played throughout the calibration until the configured
 * calibration duration expires.
 *
 * @param reading Current magnetometer measurement.
 * @param eeprom EEPROM controller used to store calibration data.
 * @param buzzer Buzzer used for user feedback.
 */
void MagnetometerSensor::calib_reading(Magnetometer& reading, EEPROMController& eeprom, BuzzerController& buzzer) {

    // Update the observed extrema for each axis.
    if(reading.mx > _calib_max_axis.mx) _calib_max_axis.mx = reading.mx;
    if(reading.mx < _calib_min_axis.mx) _calib_min_axis.mx = reading.mx;

    if(reading.my > _calib_max_axis.my) _calib_max_axis.my = reading.my;
    if(reading.my < _calib_min_axis.my) _calib_min_axis.my = reading.my;

    if(reading.mz > _calib_max_axis.mz) _calib_max_axis.mz = reading.mz;
    if(reading.mz < _calib_min_axis.mz) _calib_min_axis.mz = reading.mz;

    // Accumulate the average magnetic field magnitude.
    double mag = sqrtf(reading.mx * reading.mx +
                       reading.my * reading.my +
                       reading.mz * reading.mz);

    _calib_magnitude_sum += mag;
    _calib_num_datapoints++;

    // Periodically remind the user that calibration is still active.
    if(get_time_since_calibration_start() > 15000 * _calib_beeping && _calib_beeping < 4) {
        _calib_beeping++;
        buzzer.play_tune(mg_calib_inp, C_MG_LENGTH);
    }

    // Complete calibration once the allotted time expires.
    if(get_time_since_calibration_start() > _calib_time) {
        Serial.println("[MAG] Calibration done.");
        commit_calibration(eeprom, buzzer);
    }
}

/**
 * @brief Validates a computed magnetometer calibration.
 *
 * Performs basic sanity checks on the calculated soft-iron scale factors
 * to reject obviously invalid calibration results.
 *
 * @param b Computed hard-iron bias.
 * @param s Computed soft-iron scale factors.
 *
 * @return True if the calibration appears valid.
 */
bool MagnetometerSensor::calibration_valid(const Magnetometer& b, const Magnetometer& s) {

    constexpr float kSensorEpsilon = 1e-6f;
    constexpr float kSensorMaxShear = 5.0f;

    // Reject nearly-zero scale factors.
    if(s.mx < kSensorEpsilon || s.my < kSensorEpsilon || s.mz < kSensorEpsilon) {
        return false;
    }

    float s_min = fminf(s.mx, fminf(s.my, s.mz));
    float s_max = fmaxf(s.mx, fmaxf(s.my, s.mz));

    // Large differences between scale factors may indicate poor coverage
    // during calibration.
    if(s_max > kSensorMaxShear * s_min) {
        // Reserved for future validation.
        // return false;
    }

    return true;
}

/**
 * @brief Restores the previously saved magnetometer calibration.
 *
 * Exits calibration mode and reloads the stored hard-iron and soft-iron
 * calibration values from EEPROM.
 *
 * @param eeprom EEPROM controller containing the saved calibration.
 */
void MagnetometerSensor::restore_calibration(EEPROMController& eeprom) {

    in_calibration_mode = false;

    // Restore the saved calibration values.
    calibration_bias_softiron = eeprom.data.mmc5983ma_softiron_bias;
    calibration_bias_hardiron = eeprom.data.mmc5983ma_hardiron_bias;
}

/**
 * @brief Finalizes and stores the magnetometer calibration.
 *
 * Computes the hard-iron and soft-iron correction values from the
 * collected calibration data, validates the result, and saves the
 * calibration to EEPROM if successful.
 *
 * @param eeprom EEPROM controller used to store the calibration.
 * @param buzzer Buzzer used to indicate success or failure.
 */
void MagnetometerSensor::commit_calibration(EEPROMController& eeprom, BuzzerController& buzzer) {

    in_calibration_mode = false;

    Magnetometer b, s;

    // Compute the average magnetic field magnitude.
    double magnitude_tgt = _calib_magnitude_sum / _calib_num_datapoints;

    // Compute the hard-iron bias (offset).
    b.mx = (_calib_min_axis.mx + _calib_max_axis.mx) / 2;
    b.my = (_calib_min_axis.my + _calib_max_axis.my) / 2;
    b.mz = (_calib_min_axis.mz + _calib_max_axis.mz) / 2;

    // Compute the soft-iron scale factors.
    s.mx = (_calib_max_axis.mx - _calib_min_axis.mx) / (2 * magnitude_tgt);
    s.my = (_calib_max_axis.my - _calib_min_axis.my) / (2 * magnitude_tgt);
    s.mz = (_calib_max_axis.mz - _calib_min_axis.mz) / (2 * magnitude_tgt);

    Serial.print("MAG: ");
    Serial.println(magnitude_tgt);

    // Reject invalid calibration results.
    if(!calibration_valid(b, s)) {
        Serial.println("[MAG] Calibration is bad.");
        buzzer.play_tune(mg_calib_bad, C_MG_LENGTH);
        return;
    }

    // Store the new calibration.
    calibration_bias_hardiron = b;
    calibration_bias_softiron = s;

    buzzer.play_tune(mg_calib_done, C_MG_LENGTH);

    eeprom.data.mmc5983ma_softiron_bias = calibration_bias_softiron;
    eeprom.data.mmc5983ma_hardiron_bias = calibration_bias_hardiron;
    eeprom.commit();
}