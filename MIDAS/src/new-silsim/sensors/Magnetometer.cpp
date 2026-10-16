#include "simulated_sensors.h"

// TODO see if non init and non read functions need rewriting

#define MGC_TONE_PITCH_H Sound{3000, 50}
#define MGC_TONE_PITCH_L Sound{2200, 50}
#define MGC_TONE_PITCH_LONG Sound{3000, 250}
#define MGC_TONE_PITCH_LONG_L Sound{2000, 250}
#define MGC_TONE_WAIT Sound{0, 50}
#define MGC_TONE_NOOP Sound{0, 1}

Sound mg_calib_rdy[C_MG_LENGTH] = {MGC_TONE_PITCH_H, MGC_TONE_WAIT, MGC_TONE_PITCH_H, MGC_TONE_WAIT, MGC_TONE_PITCH_H};
Sound mg_calib_done[C_MG_LENGTH] = {MGC_TONE_PITCH_LONG, MGC_TONE_WAIT, MGC_TONE_PITCH_H, MGC_TONE_WAIT, MGC_TONE_PITCH_H};
Sound mg_calib_inp[C_MG_LENGTH] = {MGC_TONE_PITCH_H, MGC_TONE_NOOP, MGC_TONE_NOOP, MGC_TONE_NOOP,MGC_TONE_NOOP};
Sound mg_calib_bad[C_MG_LENGTH] = {MGC_TONE_PITCH_LONG_L, MGC_TONE_WAIT, MGC_TONE_PITCH_LONG_L, MGC_TONE_NOOP, MGC_TONE_NOOP};



ErrorCode MagnetometerSensor::init() {
    silsim_init("magnetometer");

    return ErrorCode::NoError;
}

Magnetometer MagnetometerSensor::read() {
	std::map<std::string, double> data = silsim_read("magnetometer")
	Magnetometer reading{data["mX"], data["my"], data["mz"]}
}


void MagnetometerSensor::begin_calibration(BuzzerController& buzzer) {
    Serial.println("[MAG] Calibration begin");
    buzzer.play_tune(mg_calib_rdy, C_MG_LENGTH);
    in_calibration_mode = true;
    _calib_begin_timestamp = millis();
    _calib_beeping = 1;
    _calib_max_axis = {-INFINITY, -INFINITY, -INFINITY};
    _calib_min_axis = {INFINITY, INFINITY, INFINITY};
    _calib_magnitude_sum = 0.0;
    _calib_num_datapoints = 0;
}

void MagnetometerSensor::calib_reading(Magnetometer& reading, EEPROMController& eeprom, BuzzerController& buzzer) {
    // X reading
    if(reading.mx > _calib_max_axis.mx) { _calib_max_axis.mx = reading.mx; };
    if(reading.mx < _calib_min_axis.mx) { _calib_min_axis.mx = reading.mx; };
    // Y
    if(reading.my > _calib_max_axis.my) { _calib_max_axis.my = reading.my; };
    if(reading.my < _calib_min_axis.my) { _calib_min_axis.my = reading.my; };
    // Z
    if(reading.mz > _calib_max_axis.mz) { _calib_max_axis.mz = reading.mz; };
    if(reading.mz < _calib_min_axis.mz) { _calib_min_axis.mz = reading.mz; };

    // Magnitude
    double mag = sqrtf(reading.mx*reading.mx + reading.my*reading.my + reading.mz*reading.mz);
    _calib_magnitude_sum += mag;
    _calib_num_datapoints++;

    // Beeping
    if(get_time_since_calibration_start() > 15000 * _calib_beeping && _calib_beeping < 4) {
        _calib_beeping++;
        buzzer.play_tune(mg_calib_inp, C_MG_LENGTH);
    }

    if(get_time_since_calibration_start() > _calib_time) {
        Serial.println("[MAG] Calibration done.");
        commit_calibration(eeprom, buzzer);
    }
}

bool MagnetometerSensor::calibration_valid(const Magnetometer& b, const Magnetometer& s) {
    constexpr float kSensorEpsilon = 1e-6f;
    constexpr float kSensorMaxShear = 5.0f; // probably should tweak this

    if(s.mx < kSensorEpsilon || s.my < kSensorEpsilon || s.mz < kSensorEpsilon) {
        // Scale data is too small, so we probably didn't get enough data
        return false;
    }

    float s_min = fminf(s.mx, fminf(s.my, s.mz)); // Get minimum value of s
    float s_max = fmaxf(s.mx, fmaxf(s.my, s.mz)); // Get minimum value of s

    if(s_max > kSensorMaxShear * s_min) {
        // this is iffy but could indicate strong sensor reference ellipsoid shear
        // return false;
    }

    return true;
}

void MagnetometerSensor::restore_calibration(EEPROMController& eeprom) {
    in_calibration_mode = false;

    // Read back the calibration data from EEPROM:
    calibration_bias_softiron = eeprom.data.mmc5983ma_softiron_bias;
    calibration_bias_hardiron = eeprom.data.mmc5983ma_hardiron_bias;
}

void MagnetometerSensor::commit_calibration(EEPROMController& eeprom, BuzzerController& buzzer) {
    in_calibration_mode = false;

    Magnetometer b, s;
    double magnitude_tgt = _calib_magnitude_sum / _calib_num_datapoints;

    // b: Origin offset
    b.mx = (_calib_min_axis.mx + _calib_max_axis.mx) / 2;
    b.my = (_calib_min_axis.my + _calib_max_axis.my) / 2;
    b.mz = (_calib_min_axis.mz + _calib_max_axis.mz) / 2;

    // s: Scale offset
    s.mx = (_calib_max_axis.mx - _calib_min_axis.mx) / (2*magnitude_tgt);
    s.my = (_calib_max_axis.my - _calib_min_axis.my) / (2*magnitude_tgt);
    s.mz = (_calib_max_axis.mz - _calib_min_axis.mz) / (2*magnitude_tgt);

    Serial.print("MAG: ");
    Serial.println(magnitude_tgt);

    if(!calibration_valid(b, s)) {
        Serial.println("[MAG] Calibration is bad.");
        buzzer.play_tune(mg_calib_bad, C_MG_LENGTH);
        return; // The calibration is garbage... this probably shouldn't fail silently but idc rn.
    }

    calibration_bias_hardiron = b;
    calibration_bias_softiron = s;
    buzzer.play_tune(mg_calib_done, C_MG_LENGTH);

    eeprom.data.mmc5983ma_softiron_bias = calibration_bias_softiron;
    eeprom.data.mmc5983ma_hardiron_bias = calibration_bias_hardiron;
    eeprom.commit();
}
