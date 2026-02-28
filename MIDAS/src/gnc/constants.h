#pragma once

// constants
const float pi = 3.14159268;
const float gravity_ms2 = 9.81;           // (m/s^2) accel due to gravity
const float accel_noise_density_x = 1000; // ug/sqrt(hz) from the accelerometer on MIDAS MINI. Assuming Acceleration noise density (high-g) in high-performance mode
const float accel_noise_density_y = 1000; // ug/sqrt(hz) from the accelerometer on MIDAS MINI
const float accel_noise_density_z = 1000; // ug/sqrt(hz) from the accelerometer on MIDAS MINI
const float gyro_RMS_noise =3.8; //mdps/√Hz
const float mag_noise = 0.4; //mG