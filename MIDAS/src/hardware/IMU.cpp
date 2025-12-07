#include "lsm6dsv320x_reg.h"

#include "IMUOps.h"
#include "errors.h"
#include "sensors.h"

#define ACCELEROMETER_RATE 0
#define GYRO_RATE 0

stmdev_ctx_t dev_ctx;
lsm6dsv320x_all_sources_t all_sources;
lsm6dsv320x_filt_setting_mask_t filt_settling_mask;

int16_t raw_accel[3];
int16_t raw_accel_hg[3];
int16_t raw_av[3];

IMU IMUSensor::read(){
    lsm6dsv320x_all_sources_get(&dev_ctx, &all_sources);

    IMUData reading;

    //Acceleration
    if(all_sources.drdy_xl){
        lsm6dsv320x_acceleration_raw_get(&dev_ctx, raw_accel);
        reading.highg_acceleration.ax = lsm6dsv320x_from_fs2_to_mg(raw_accel[0]) / 1000.0;
        reading.highg_acceleration.ay = lsm6dsv320x_from_fs2_to_mg(raw_accel[1]) / 1000.0;
        reading.highg_acceleration.az = lsm6dsv320x_from_fs2_to_mg(raw_accel[2]) / 1000.0;
    }

    //High-G Acceleration
    if(all_sources.drdy_xlhgda){
        lsm6dsv320x_acceleration_raw_get(&dev_ctx, raw_accel_hg);
        reading.highg_acceleration.ax = lsm6dsv320x_from_fs2_to_mg(raw_accel_hg[0]) / 1000.0;
        reading.highg_acceleration.ay = lsm6dsv320x_from_fs2_to_mg(raw_accel_hg[1]) / 1000.0;
        reading.highg_acceleration.az = lsm6dsv320x_from_fs2_to_mg(raw_accel_hg[2]) / 1000.0;
    }

    //Angular rate
    if(all_sources.drdy_gy){
        lsm6dsv320x_angular_rate_raw_get(&dev_ctx, raw_av);
        reading.angular_velocity.gx = lsm6dsv320x_from_fs2000_to_mdps(raw_av[0]) / 1000.0;
        reading.angular_velocity.gy = lsm6dsv320x_from_fs2000_to_mdps(raw_av[1]) / 1000.0;
        reading.angular_velocity.gz = lsm6dsv320x_from_fs2000_to_mdps(raw_av[2]) / 1000.0;
    }

    return reading;
}

ErrorCode IMUSensor::init(){
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = NULL;
    dev_ctx.handle = &SENSOR_BUS;

    //platform_delay(BOOT_TIME)? Probably not.

    int8_t whoamI;
    lsm6dsv320x_device_id_get(&dev_ctx, &whomaI);
    if(whoamI != SM6DSV_ID)
        return LSMCouldNotBeInitialized;

    //Default config (?)
    lsm6dsv320x_sw_por(&dev_ctx);

    //Set data rate & scale
    lsm6dsv320x_xl_data_rate_set(&dev_ctx, lsm6dsv320x_ODR_AT_7Hz5);
    lsm6dsv320x_gy_data_rate_set(&dev_ctx, lsm6dsv320x_ODR_AT_15Hz);

    lsm6dsv320x_xl_full_scale_set(&dev_ctx, lsm6dsv320x_2g);
    lsm6dsv320x_gy_full_scale_set(&dev_ctx, lsm6dsv320x_2000dps);

    //Filter chain:Copied from provided example?????? 
    //TODO: UNDERSTAND OR CHANGE
    filt_settling_mask.drdy = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g = PROPERTY_ENABLE;
    lsm6dsv320x_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
    lsm6dsv320x_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv320x_filt_gy_lp1_bandwidth_set(&dev_ctx, lsm6dsv320x_GY_ULTRA_LIGHT);
    lsm6dsv320x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv320x_filt_xl_lp2_bandwidth_set(&dev_ctx, lsm6dsv320x_XL_STRONG);
}