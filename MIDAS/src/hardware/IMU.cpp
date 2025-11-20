#include "lsm6dsv_reg.h"

#include "IMUOps.h"
#include "errors.h"

#define ACCELEROMETER_RATE 0
#define GYRO_RATE 0

stmdev_ctx_t dev_ctx;
lsm6dsv_all_sources_t all_sources;

struct IMU{ 
};

struct IMUSensor{
    ErrorCode init();
    IMU read();
};

IMU IMUSensor::read(){
    lsm6dsv_all_sources_get(&dev_ctx, &all_sources);

    //Acceleration
    if(all_sources.drdy_xl){
        lsm6dsv_acceleration_raw_get(&dev_ctx, );
    }
    
    //Angular rate
    if(all_sources.drdy_gy){
        lsm6dsv_angular_rate_raw_get(&dev_ctx, );
        
    }
}

ErrorCode IMUSensor::init(){
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = NULL;
    //dev_ctx.handle = &SENSOR_BUS;


    //platform_delay(BOOT_TIME)? Probably not.
    //lsm6dsv_device_id_get - verify device id


    //Set data rate

    //Set "full scale"?

    //Filtering chain?


}