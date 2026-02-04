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
        reading.highg_acceleration.ax = LSM6DSV.from_fs64_to_mg(raw_accel[0]) / 1000.0;
        reading.highg_acceleration.ay = LSM6DSV.from_fs64_to_mg(raw_accel[1]) / 1000.0;
        reading.highg_acceleration.az = LSM6DSV.from_fs64_to_mg(raw_accel[2]) / 1000.0;
    }

    //High-G Acceleration
    if(status.xlhgda){
        LSM6DSV.hg_acceleration_raw_get(raw_accel_hg);
        reading.highg_acceleration.ax = LSM6DSV.from_fs64_to_mg(raw_accel_hg[0]) / 1000.0;//Edits needed? (Check)
        reading.highg_acceleration.ay = LSM6DSV.from_fs64_to_mg(raw_accel_hg[1]) / 1000.0;
        reading.highg_acceleration.az = LSM6DSV.from_fs64_to_mg(raw_accel_hg[2]) / 1000.0;
    }

    //Angular rate
    if(status.gda){
        LSM6DSV.angular_rate_raw_get(raw_av);
        reading.angular_velocity.vx = LSM6DSV.from_fs2000_to_mdps(raw_av[0]) / 1000.0;//Edits needed? 
        reading.angular_velocity.vy = LSM6DSV.from_fs2000_to_mdps(raw_av[1]) / 1000.0;
        reading.angular_velocity.vz = LSM6DSV.from_fs2000_to_mdps(raw_av[2]) / 1000.0;
    }


    
    //Embedded SFLP
    uint16_t val[4]; 

	LSM6DSV.lsm6dsv320x_sflp_quaternion_raw_get((int16_t*)&val);//4 elements

    reading.hw_filtered.quaternion.w = LSM6DSV.from_sflp_to_mg(val[0]);
    reading.hw_filtered.quaternion.x = LSM6DSV.from_sflp_to_mg(val[1]);
    reading.hw_filtered.quaternion.y = LSM6DSV.from_sflp_to_mg(val[2]);
    reading.hw_filtered.quaternion.z = LSM6DSV.from_sflp_to_mg(val[3]);

    LSM6DSV.sflp_gbias_raw_get((int16_t*)&val);//3 elements
    for(int i = 0; i<3; i++)
        reading.hw_filtered.gbias[i] = LSM6DSV.from_sflp_to_mg(val[i]);


    LSM6DSV.sflp_gravity_raw_get((int16_t*)&val);//3 elements
    for(int i = 0; i<3; i++)
        reading.hw_filtered.gravity[i] = LSM6DSV.from_sflp_to_mg(val[i]);

    return reading;
}

ErrorCode IMUSensor::init(){
    uint8_t whoami;
    LSM6DSV.device_id_get(&whoami);
    if(whoami != LSM6DSV320X_ID) 
        return IMUCouldNotBeInitialized;


    //?????
    LSM6DSV.sw_por();
    
    // the second parameter used to be normal instead of high-performance
    LSM6DSV.xl_setup(LSM6DSV320X_ODR_AT_7Hz5, LSM6DSV320X_XL_HIGH_PERFORMANCE_MD);
    LSM6DSV.gy_setup(LSM6DSV320X_ODR_AT_15Hz, LSM6DSV320X_GY_HIGH_PERFORMANCE_MD);
    LSM6DSV.hg_xl_data_rate_set(LSM6DSV320X_HG_XL_ODR_AT_960Hz, 1);//xl_setup only handles lowg, this should also set the enable register
    
    LSM6DSV.hg_xl_full_scale_set(LSM6DSV320X_64g);//highg scale set
    LSM6DSV.xl_full_scale_set(LSM6DSV320X_2g);//lowg scale set
    LSM6DSV.gy_full_scale_set(LSM6DSV320X_2000dps);
    
    LSM6DSV.sflp_enable_set(1);

    //Filter initialization (I really have no idea)
    LSM6DSV.filt_settling_mask_set(false, false, false);

    // Low-pass filters:
    LSM6DSV.filt_gy_lp1_set(PROPERTY_DISABLE);
    //lsm6dsv320x_filt_gy_lp1_bandwidth_set(&dev_ctx, lsm6dsv320x_GY_ULTRA_LIGHT);
    LSM6DSV.filt_xl_lp2_set(PROPERTY_DISABLE);
    //lsm6dsv320x_filt_xl_lp2_bandwidth_set(&dev_ctx, lsm6dsv320x_XL_STRONG);


    return NoError;
}