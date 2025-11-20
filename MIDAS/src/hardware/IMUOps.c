#include "stddef.h"

#include "Wire.h"
#include "lsm6dsv_reg.h"
#include "esp32-hal-i2c.h"
#include "IMUOps.h"

//?: extern "C" i2c_read i2cRead (Wire library is c++)

#define I2C_NUM 0
#define TIME_OUT_MS 25

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len){
    size_t num_reads = 1;
    esp_err_t result = i2cRead(I2C_NUM, reg, bufp,len,TIME_OUT_MS, &num_reads);
    
    if(result == ESP_OK) return 0;
    return -1;
}

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len){
    return -1;
}