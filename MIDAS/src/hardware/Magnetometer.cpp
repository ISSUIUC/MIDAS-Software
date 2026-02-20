#include <SparkFun_MMC5983MA_Arduino_Library.h>

#include "sensors.h"
#include "hal.h"

SFE_MMC5983MA MMC5983;           // global static instance of the sensor

ErrorCode MagnetometerSensor::init() {
    // Checks if sensor is connected
    if (!MMC5983.begin(MMC5983_CS)) {
        return ErrorCode::MagnetometerCouldNotBeInitialized;
    }
    return ErrorCode::NoError;
}

Magnetometer MagnetometerSensor::read() {
    // read from aforementioned global instance of sensor
    uint32_t cx, cy, cz;
    double X, Y, Z;
    
    MMC5983.getMeasurementXYZ(&cx, &cy, &cz);
    
    // The magnetic field values are 18-bit unsigned. The _approximate_ zero (mid) point is 2^17
    // Here we scale each field to +/- 1.0 to make it easier to convert to Gauss
    // https://github.com/sparkfun/SparkFun_MMC5983MA_Magnetometer_Arduino_Library/tree/main/examples
    double sf = (double)(1 << 17);
    X = ((double)cx - sf)/sf;
    Y = ((double)cy - sf)/sf;
    Z = ((double)cz - sf)/sf;
    
    Magnetometer reading{X, Y, Z};
    return reading;
}

void MagnetometerSensor::begin_calibration() {
    in_calibration_mode = true;
    _calib_begin_timestamp = millis();
    _calib_max_axis = {0.0, 0.0, 0.0};
    _calib_min_axis = {0.0, 0.0, 0.0};
}

void MagnetometerSensor::calib_reading(Magnetometer& reading, EEPROMController& eeprom) {
    // X reading
    if(reading.mx > _calib_max_axis.mx) { _calib_max_axis.mx = reading.mx; };
    if(reading.mx < _calib_min_axis.mx) { _calib_min_axis.mx = reading.mx; };
    // Y
    if(reading.my > _calib_max_axis.my) { _calib_max_axis.my = reading.my; };
    if(reading.my < _calib_min_axis.my) { _calib_min_axis.my = reading.my; };
    // Z
    if(reading.mz > _calib_max_axis.mz) { _calib_max_axis.mz = reading.mz; };
    if(reading.mz < _calib_min_axis.mz) { _calib_min_axis.mz = reading.mz; };

    if(get_time_since_calibration_start() > _calib_time) {
        commit_calibration(eeprom);
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
        return false;
    }

    return true;
}

void MagnetometerSensor::restore_calibration(EEPROMController& eeprom) {
    in_calibration_mode = false;

    // Read back the calibration data from EEPROM:
    calibration_bias_softiron = eeprom.data.mmc5983ma_softiron_bias;
    calibration_bias_hardiron = eeprom.data.mmc5983ma_hardiron_bias;
}

void MagnetometerSensor::commit_calibration(EEPROMController& eeprom) {
    in_calibration_mode = false;

    Magnetometer b, s;

    // b: Origin offset
    b.mx = (_calib_min_axis.mx + _calib_max_axis.mx) / 2;
    b.my = (_calib_min_axis.my + _calib_max_axis.my) / 2;
    b.mz = (_calib_min_axis.mz + _calib_max_axis.mz) / 2;

    // s: Scale offset
    s.mx = (_calib_min_axis.mx - _calib_max_axis.mx) / 2;
    s.my = (_calib_min_axis.my - _calib_max_axis.my) / 2;
    s.mz = (_calib_min_axis.mz - _calib_max_axis.mz) / 2;

    if(!calibration_valid(b, s)) {
        return; // The calibration is garbage... this probably shouldn't fail silently but idc rn.
    }

    calibration_bias_hardiron = b;
    calibration_bias_softiron = s;

    eeprom.data.mmc5983ma_softiron_bias = calibration_bias_softiron;
    eeprom.data.mmc5983ma_hardiron_bias = calibration_bias_hardiron;
    eeprom.commit();
}