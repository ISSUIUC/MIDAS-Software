#include "stddef.h"

#include "Wire.h"
#include "lsm6dsv_reg.h"
#include "esp32-hal-i2c.h"

#define I2C_NUM 0
#define TIME_OUT_MS 25

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);