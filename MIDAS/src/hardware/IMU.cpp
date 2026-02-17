#include "lsm6dsv320x.h"

#include "errors.h"
#include "sensors.h" 

#define NUM_DIRECTIONS 3

LSM6DSV320XClass LSM6DSV(SPI, IMU_CS_PIN, IMU_IRQ_PIN);

int16_t raw_accel[NUM_DIRECTIONS];
int16_t raw_accel_hg[NUM_DIRECTIONS];
int16_t raw_av[NUM_DIRECTIONS];

unsigned long lastTime = 0;
float deltaTime = 0;

IMU IMUSensor::read(){
    lsm6dsv320x_status_reg_t status = LSM6DSV.get_status();

    IMU reading{};

    //Low-G Acceleration
    if(status.xlda){
        LSM6DSV.get_lowg_acceleration_from_fs8_to_g(&reading.lowg_acceleration.ax,
                                                    &reading.lowg_acceleration.ay,
                                                    &reading.lowg_acceleration.az);
    }

    //High-G Acceleration
    if(status.xlhgda){
        LSM6DSV.get_highg_acceleration_from_fs64_to_g(&reading.highg_acceleration.ax,
                                                    &reading.highg_acceleration.ay,
                                                    &reading.highg_acceleration.az);
    }

    //Angular rate
    if(status.gda){
        //Serial.println("oh we are angular");
        LSM6DSV.get_angular_velocity_from_fs2000_to_dps(&reading.angular_velocity.vx, 
                                                    &reading.angular_velocity.vy, 
                                                    &reading.angular_velocity.vz);
    }

    return reading;
}

IMU_SFLP IMUSensor::read_sflp() {

    IMU_SFLP reading;

    uint16_t val[4];

	LSM6DSV.lsm6dsv320x_sflp_quaternion_raw_get(val);//4 elements

    reading.quaternion.w = LSM6DSV.sflp_quaternion_raw_to_float(val[0]);//Will have to find a half to single precision conversion function somewhere
    reading.quaternion.x = LSM6DSV.sflp_quaternion_raw_to_float(val[1]);
    reading.quaternion.y = LSM6DSV.sflp_quaternion_raw_to_float(val[2]);
    reading.quaternion.z = LSM6DSV.sflp_quaternion_raw_to_float(val[3]);

    //(Feature) UPDATE TO USE FIFO -> If the readings are currently okay, this wont be a priority. Circular Buffer FIFO will be a feature
    LSM6DSV.sflp_gbias_raw_get((int16_t*)&val);//3 elements

    reading.gyro_bias.vx = LSM6DSV.sflp_gbias_raw_to_mdps(val[0]) / 1000.0;
    reading.gyro_bias.vy = LSM6DSV.sflp_gbias_raw_to_mdps(val[1]) / 1000.0;
    reading.gyro_bias.vz = LSM6DSV.sflp_gbias_raw_to_mdps(val[2]) / 1000.0;

    //UPDATE TO USE FIFO -> If the readings are currently okay, Circular Buffer FIFO will be a feature
    LSM6DSV.sflp_gravity_raw_get((int16_t*)&val);//3 elements
    
    reading.gravity.ax = LSM6DSV.sflp_gravity_raw_to_mg(val[0]) / 1000.0;
    reading.gravity.ay = LSM6DSV.sflp_gravity_raw_to_mg(val[1]) / 1000.0;
    reading.gravity.az = LSM6DSV.sflp_gravity_raw_to_mg(val[2]) / 1000.0;
    
    return reading;
}

//dont need this anymore

// AngularKalmanData IMUSensor::read_Kalman_Angular() {
//     sh2_SensorValue_t event;
//     Vec3 euler;
//     Vec3 filtered_euler = {0, 0, 0};
//     const float alpha = 0.98; // Higher values dampen out current measurements --> reduce peaks
//     unsigned long currentTime = millis();
//     deltaTime = (currentTime - lastTime) / 1000.0;
//     lastTime = currentTime;
//     if (imu.getSensorEvent(&event)) {
//         AngularKalmanData sensor_reading;
//         sensor_reading.has_data = true;
//         if (event.sensorId == SH2_ARVR_STABILIZED_RV) {
//             euler = quaternionToEulerRV(&event.un.arvrStabilizedRV, true);
//             sensor_reading.reading_type = OrientationReadingType::FULL_READING;
//             sensor_reading.quaternion.w = event.un.arvrStabilizedRV.real;
//             sensor_reading.quaternion.x = event.un.arvrStabilizedRV.i;
//             sensor_reading.quaternion.y = event.un.arvrStabilizedRV.j;
//             sensor_reading.quaternion.z = event.un.arvrStabilizedRV.k;
//             break;
//         }
//         sensor_reading.yaw = -euler.y;
//         sensor_reading.pitch = euler.x;
//         sensor_reading.roll = euler.z;
//         if (initial_flag == 0)
//         {
//             initial_orientation = sensor_reading;
//             initial_flag = 1;
//         }
//     }
// }


ErrorCode IMUSensor::init(){
    uint8_t whoami;

    

    LSM6DSV.device_id_get(&whoami);
    if(whoami != LSM6DSV320X_ID) 
        return IMUCouldNotBeInitialized;

    LSM6DSV.sw_por();

    // the second parameter used to be normal instead of high-performance
    LSM6DSV.xl_setup(LSM6DSV320X_ODR_AT_7Hz5, LSM6DSV320X_XL_HIGH_PERFORMANCE_MD);
    LSM6DSV.gy_setup(LSM6DSV320X_ODR_AT_15Hz, LSM6DSV320X_GY_HIGH_PERFORMANCE_MD);
    LSM6DSV.hg_xl_data_rate_set(LSM6DSV320X_HG_XL_ODR_AT_960Hz, 1);//xl_setup only handles lowg, this should also set the enable register
    
    LSM6DSV.hg_xl_full_scale_set(LSM6DSV320X_64g);//highg scale set
    LSM6DSV.xl_full_scale_set(LSM6DSV320X_8g);//low scale set
    LSM6DSV.gy_full_scale_set(LSM6DSV320X_2000dps);
    
    LSM6DSV.sflp_enable_set(1);

    //Filter initialization (I really have no idea)
    LSM6DSV.filt_settling_mask_set(false, false, false);

    // Low-pass filters:
    LSM6DSV.filt_gy_lp1_set(PROPERTY_DISABLE);
    //lsm6dsv320x_filt_gy_lp1_bandwidth_set(&dev_ctx, lsm6dsv320x_GY_ULTRA_LIGHT);
    LSM6DSV.filt_xl_lp2_set(PROPERTY_DISABLE);

    return NoError;
}