#include "lsm6dsv320x.h"

#include "errors.h"
#include "sensors.h"

#define IMU_CS_PIN 39
#define IMU_IRQ_PIN 19

LSM6DSV320XClass LSM6DSV(SPI, IMU_CS_PIN, IMU_IRQ_PIN);

int16_t raw_accel[3];
int16_t raw_accel_hg[3];
int16_t raw_av[3];

IMU IMUSensor::read(){
    lsm6dsv320x_status_reg_t status = LSM6DSV.get_status();

    IMU reading{};

    //Acceleration
    if(status.xlda){
        LSM6DSV.acceleration_raw_get(raw_accel);
        reading.highg_acceleration.ax = LSM6DSV.from_fs2_to_mg(raw_accel[0]) / 1000.0;
        reading.highg_acceleration.ay = LSM6DSV.from_fs2_to_mg(raw_accel[1]) / 1000.0;
        reading.highg_acceleration.az = LSM6DSV.from_fs2_to_mg(raw_accel[2]) / 1000.0;
    }

    //High-G Acceleration
    if(status.xlhgda){
        LSM6DSV.hg_acceleration_raw_get(raw_accel_hg);
        reading.highg_acceleration.ax = LSM6DSV.from_fs2_to_mg(raw_accel_hg[0]) / 1000.0;
        reading.highg_acceleration.ay = LSM6DSV.from_fs2_to_mg(raw_accel_hg[1]) / 1000.0;
        reading.highg_acceleration.az = LSM6DSV.from_fs2_to_mg(raw_accel_hg[2]) / 1000.0;
    }

    //Angular rate
    if(status.gda){
        LSM6DSV.angular_rate_raw_get(raw_av);
        reading.angular_velocity.vx = LSM6DSV.from_fs2000_to_mdps(raw_av[0]) / 1000.0;
        reading.angular_velocity.vy = LSM6DSV.from_fs2000_to_mdps(raw_av[1]) / 1000.0;
        reading.angular_velocity.vz = LSM6DSV.from_fs2000_to_mdps(raw_av[2]) / 1000.0;
    }

    return reading;
}

ErrorCode IMUSensor::init(){
    //platform_delay(BOOT_TIME)? Probably not.

    uint8_t whoamI;
    LSM6DSV.device_id_get(&whoamI);
    if(whoamI != LSM6DSV320X_ID)
        return IMUCouldNotBeInitialized;

    //Default config (?)
    LSM6DSV.sw_por();

    //Set data rate & scale
    LSM6DSV.xl_setup(LSM6DSV320X_ODR_AT_7Hz5, LSM6DSV320X_XL_NORMAL_MD);
    LSM6DSV.gy_setup(LSM6DSV320X_ODR_AT_15Hz, LSM6DSV320X_GY_HIGH_PERFORMANCE_MD);

    LSM6DSV.xl_full_scale_set(LSM6DSV320X_8g);
    LSM6DSV.gy_full_scale_set(LSM6DSV320X_2000dps);

    //Filter initialization (I really have no idea)

    LSM6DSV.filt_settling_mask_set(false, false, false);

    // Low-pass filters:
    LSM6DSV.filt_gy_lp1_set(PROPERTY_DISABLE);
    //lsm6dsv320x_filt_gy_lp1_bandwidth_set(&dev_ctx, lsm6dsv320x_GY_ULTRA_LIGHT);
    LSM6DSV.filt_xl_lp2_set(PROPERTY_DISABLE);
    //lsm6dsv320x_filt_xl_lp2_bandwidth_set(&dev_ctx, lsm6dsv320x_XL_STRONG);

    return NoError;
}