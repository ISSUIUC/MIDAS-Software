#include "lsm6dsv320x.h"

#include "Arduino.h"
#include <SPI.h>

LSM6DSV320XClass::LSM6DSV320XClass(SPIClass& spi, int csPin, int irqPin):
    _spi(&spi), _csPin(csPin), _irqPin(irqPin), _spiSettings(8E6, MSBFIRST, SPI_MODE0)
{
}

void LSM6DSV320XClass::bytecpy(uint8_t *target, uint8_t *source){
    if ((target != nullptr) && (source != nullptr))
        *target = *source;
}

int32_t LSM6DSV320XClass::read_reg(uint8_t reg, uint8_t *data, uint16_t len){
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(0x80 | reg);
    _spi->transfer(data, len);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();

    return 0;
}

int32_t LSM6DSV320XClass::write_reg(uint8_t reg,
                                     uint8_t *data,
                                     uint16_t len){

    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg);
    _spi->transfer(*data);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();

    return 0;
}

lsm6dsv320x_status_reg_t LSM6DSV320XClass::get_status(){
    lsm6dsv320x_status_reg_t rv{};
    read_reg(LSM6DSV320X_STATUS_REG, (uint8_t*)&rv, 1);

    return rv;
}

/**
  * @brief  Linear acceleration sensor.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Linear acceleration sensor.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LSM6DSV320XClass::acceleration_raw_get(int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret = read_reg(LSM6DSV320X_OUTX_L_A, &buff[0], 6);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}

/**
  * @brief  Linear acceleration sensor for hg channel mode.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Linear acceleration sensor or High-G channel mode.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LSM6DSV320XClass::hg_acceleration_raw_get(int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret = read_reg(LSM6DSV320X_UI_OUTX_L_A_OIS_HG, &buff[0], 6);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}

int32_t LSM6DSV320XClass::angular_rate_raw_get(int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret = read_reg(LSM6DSV320X_OUTX_L_G, &buff[0], 6);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}


float LSM6DSV320XClass::from_fs2_to_mg(int16_t lsb) //the _reg libraries have the functions, maybe use that. We can figure out.
{
  return static_cast<float>(lsb) * 0.061f; //if you check the datasheet, you will see that for 64gs, we will have to use different conversion values.
}


float LSM6DSV320XClass::from_fs8_to_mg(int16_t lsb)
{
  return static_cast<float>(lsb) * 0.244f;
}

float LSM6DSV320XClass::from_fs16_to_mg(int16_t lsb) 
{
  return static_cast<float>(lsb) * 0.488f;
}

float LSM6DSV320XClass::from_fs64_to_mg(int16_t lsb)
{
  return static_cast<float>(lsb) * 1.952f;
}


float LSM6DSV320XClass::from_fs2000_to_mdps(int16_t lsb)
{
  return static_cast<float>(lsb) * 70.0f;
}

typedef union {
    uint32_t float_bits;
    float f;
} float_conv;

float LSM6DSV320XClass::half_to_float(uint16_t h)
{

    float_conv result;
  uint16_t h_exp = (h & 0x7c00u);
  uint32_t f_sgn = ((uint32_t)h & 0x8000u) << 16;
  switch (h_exp)
  {
    case 0x0000u:   // 0 or subnormal
    {
      uint16_t h_sig = (h & 0x03ffu);
      // Signed zero
      if (h_sig == 0)
      {
        return f_sgn;
      }
      // Subnormal
      h_sig <<= 1;
      while ((h_sig & 0x0400u) == 0)
      {
        h_sig <<= 1;
        h_exp++;
      }
      uint32_t f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
      uint32_t f_sig = ((uint32_t)(h_sig & 0x03ffu)) << 13;
      result.float_bits = f_sgn + f_exp + f_sig;
    }
    case 0x7c00u: // inf or NaN
      // All-ones exponent and a copy of the significand
      result.float_bits = f_sgn + 0x7f800000u + (((uint32_t)(h & 0x03ffu)) << 13);
    default: // normalized
      // Just need to adjust the exponent and shift
      result.float_bits = f_sgn + (((uint32_t)(h & 0x7fffu) + 0x1c000u) << 13);
  }

  return result.f;
}

float LSM6DSV320XClass::sflp_quaternion_raw_to_float(int16_t raw){
  //Will have to find a half to single precision conversion function somewhere in the codebase
  return half_to_float((uint16_t)raw);
}

float LSM6DSV320XClass::sflp_gbias_raw_to_mdps(int16_t raw){
  return static_cast<float>(raw) * 4.375f;
}

float LSM6DSV320XClass::sflp_gravity_raw_to_mg(int16_t raw){
  return static_cast<float>(raw) * 0.061f;
}

void LSM6DSV320XClass::get_lowg_acceleration_from_fs8_to_g(float *ax, float *ay, float *az) {
  int16_t raw_accel[NUM_DIRECTIONS];
  
  acceleration_raw_get(raw_accel);

  *ax = from_fs8_to_mg(raw_accel[0]) / 1000.0;
  *ay = from_fs8_to_mg(raw_accel[1]) / 1000.0;
  *az = from_fs8_to_mg(raw_accel[2]) / 1000.0;
}

void LSM6DSV320XClass::get_highg_acceleration_from_fs64_to_g(float *ax, float *ay, float *az) {
  int16_t raw_accel_hg[NUM_DIRECTIONS];
  
  hg_acceleration_raw_get(raw_accel_hg);
  
  *ax = from_fs64_to_mg(raw_accel_hg[0]) / 1000.0;
  *ay = from_fs64_to_mg(raw_accel_hg[1]) / 1000.0;
  *az = from_fs64_to_mg(raw_accel_hg[2]) / 1000.0;
}

void LSM6DSV320XClass::get_angular_velocity_from_fs2000_to_dps(float *vx, float *vy, float *vz) {
  int16_t raw_av[NUM_DIRECTIONS];
  
  angular_rate_raw_get(raw_av);
  
  *vx = from_fs2000_to_mdps(raw_av[0]) / 1000.0;
  *vy = from_fs2000_to_mdps(raw_av[1]) / 1000.0;
  *vz = from_fs2000_to_mdps(raw_av[2]) / 1000.0;
}

/**
  * @brief  Device ID.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Device ID.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LSM6DSV320XClass::device_id_get(uint8_t *val)
{
  int32_t ret;

  ret = read_reg(LSM6DSV320X_WHO_AM_I, val, 1);

  return ret;
}

/**
  * @brief  Perform power-on-reset of the device.
  *
  * @param  ctx      read / write interface definitions
  * @retval          0: power-on-reset has been performed, -1: error
  *
  */
int32_t LSM6DSV320XClass::sw_por()
{
  lsm6dsv320x_func_cfg_access_t func_cfg_access = {0};
  int32_t ret;

  /* 1. Set the SW_POR bit of the FUNC_CFG_ACCESS register to 1. */
  func_cfg_access.sw_por = 1;
  ret = write_reg(LSM6DSV320X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  if (ret != 0)
    return ret;

  /* 2. Wait 30 ms. */
  delay(30);

  return ret;
}

/**
  * @brief Sensor xl setup
  *        If both accelerometer and gyroscope are ON, `lsm6dsv320x_haodr_set` should
  *        be used to change HAODR mode; otherwise, this function will fail since
  *        HAODR is a shared bit.
  *
  * @param  ctx        read / write interface definitions
  * @param  xl_odr     lsm6dsv320x_data_rate_t
  * @param  xl_mode    lsm6dsv320x_xl_mode_t
  */
int32_t LSM6DSV320XClass::xl_setup(lsm6dsv320x_data_rate_t xl_odr,lsm6dsv320x_xl_mode_t xl_mode){
  int32_t ret;
  lsm6dsv320x_ctrl1_t ctrl1;
  lsm6dsv320x_ctrl2_t ctrl2;
  lsm6dsv320x_haodr_cfg_t haodr;
  uint8_t xl_ha = ((uint8_t) xl_odr >> 4) & 0xFU;

  // Table 10 of AN6119
  // 1.875 Hz allowed only in Low-power modes
  if (xl_odr == LSM6DSV320X_ODR_AT_1Hz875 &&
      xl_mode != LSM6DSV320X_XL_LOW_POWER_2_AVG_MD &&
      xl_mode != LSM6DSV320X_XL_LOW_POWER_4_AVG_MD &&
      xl_mode != LSM6DSV320X_XL_LOW_POWER_8_AVG_MD)
  {
    return -1;
  }
  // 7.5 Hz allowed only in normal or high-performance modes
  else if (xl_odr == LSM6DSV320X_ODR_AT_7Hz5 &&
      xl_mode != LSM6DSV320X_XL_NORMAL_MD && xl_mode != LSM6DSV320X_XL_HIGH_PERFORMANCE_MD)
  {
    return -1;
  }
  // if odr_xl bits has 4th bit enabled, low-power modes are not allowed
  else if (
    // odr >= 480 and low-power and normal mode
    ((uint8_t)xl_odr & 0x8) != 0 && ((uint8_t)xl_mode & 0x4) != 0 &&
    (xl_mode != LSM6DSV320X_XL_NORMAL_MD || // normal mode is not allowed for some data rates
     xl_odr == LSM6DSV320X_ODR_AT_3840Hz ||
     xl_odr == LSM6DSV320X_ODR_AT_7680Hz))
  {
    return -1;
  }

  if (xl_mode == LSM6DSV320X_XL_ODR_TRIGGERED_MD &&
      (xl_odr == LSM6DSV320X_ODR_AT_1Hz875 ||
       xl_odr == LSM6DSV320X_ODR_AT_7Hz5 ||
       xl_odr == LSM6DSV320X_ODR_AT_7680Hz))
  {
      return -1;
  }

  // if odr is choosed as high-accuracy value, mode should also be set in HAODR mode
  if ((xl_ha != 0 && xl_mode != LSM6DSV320X_XL_HIGH_ACCURACY_ODR_MD) ||
      (xl_ha == 0 && xl_mode == LSM6DSV320X_XL_HIGH_ACCURACY_ODR_MD))
  {
    return -1;
  }

  uint8_t buff[2];
  ret = read_reg(LSM6DSV320X_CTRL1, buff, 2);
  ret += read_reg(LSM6DSV320X_HAODR_CFG, (uint8_t *)&haodr, 1);

  bytecpy((uint8_t *)&ctrl1, &buff[0]);
  bytecpy((uint8_t *)&ctrl2, &buff[1]);

  if (ret != 0)
  {
    return ret;
  }

  // cross-checking haodr mode
  uint8_t both_on = ctrl1.odr_xl != LSM6DSV320X_ODR_OFF &&
    ctrl2.odr_g != LSM6DSV320X_ODR_OFF ? 1 : 0;

  // if both on, then haodr_sel is a shared bit. Could be changed through haodr_set API
  if (both_on && (xl_ha != haodr.haodr_sel))
  {
    return -1;
  }

  // if odr is choosed as an high-accuracy value, mode should be set in high-accuracy
  if ((xl_ha != 0 && xl_mode != LSM6DSV320X_XL_HIGH_ACCURACY_ODR_MD)) {
    return -1;
  }

  // Switching (enable/disable) HAODR mode require that all sensors must be in power-down mode.
  // Note: if both sensors are ON, lsm6dsv320x_haodr_set function must be used.
  if (haodr.haodr_sel != xl_ha &&
      ctrl1.op_mode_xl != xl_mode && // check if mode switch is required
      (xl_mode == LSM6DSV320X_XL_HIGH_ACCURACY_ODR_MD || // check if mode to set is HAODR
       ctrl1.op_mode_xl == LSM6DSV320X_XL_HIGH_ACCURACY_ODR_MD)) // check if previous mode was HAODR
  {
    ret += haodr_set(xl_odr, xl_mode, 
        static_cast<lsm6dsv320x_data_rate_t>(ctrl2.odr_g), 
        static_cast<lsm6dsv320x_gy_mode_t>(ctrl2.op_mode_g));
  }
  else
  {
    // if HAODR switch is not required, just set ctrl1 settings
    ctrl1.op_mode_xl = xl_mode;
    ctrl1.odr_xl = xl_odr;
    haodr.haodr_sel = xl_ha;
    ret += write_reg(LSM6DSV320X_CTRL1, (uint8_t *)&ctrl1, 1);
    ret += write_reg(LSM6DSV320X_HAODR_CFG, (uint8_t *)&haodr, 1);
  }

exit:
  return ret;
}

/**
  * @brief Sensor gy setup
  *        If both accelerometer and gyroscope are ON, `lsm6dsv320x_haodr_set` should
  *        be used to change HAODR mode; otherwise, this function will fail since
  *        HAODR is a shared bit.
  *
  * @param  ctx        read / write interface definitions
  * @param  gy_odr     lsm6dsv320x_data_rate_t
  * @param  gy_mode    lsm6dsv320x_gy_mode_t
  */
int32_t LSM6DSV320XClass::gy_setup(lsm6dsv320x_data_rate_t gy_odr, lsm6dsv320x_gy_mode_t gy_mode){
  int32_t ret;
  lsm6dsv320x_ctrl1_t ctrl1;
  lsm6dsv320x_ctrl2_t ctrl2;
  lsm6dsv320x_haodr_cfg_t haodr;
  uint8_t gy_ha = ((uint8_t) gy_odr >> 4) & 0xFU;

  // Table 13 of AN6119
  // 7.5Hz with HAODR mode enable, is already handled by the enum selection
  if (((uint8_t)gy_odr & 0x8) != 0 && gy_mode == LSM6DSV320X_GY_LOW_POWER_MD)
  {
    return -1;
  }

  if (gy_mode == LSM6DSV320X_GY_ODR_TRIGGERED_MD &&
      (gy_odr == LSM6DSV320X_ODR_AT_7Hz5 ||
       gy_odr == LSM6DSV320X_ODR_AT_7680Hz))
  {
      return -1;
  }

  // if odr is choosed as high-accuracy value, mode should also be set in HAODR mode
  if ((gy_ha != 0 && gy_mode != LSM6DSV320X_GY_HIGH_ACCURACY_ODR_MD) ||
      (gy_ha == 0 && gy_mode == LSM6DSV320X_GY_HIGH_ACCURACY_ODR_MD))
  {
    return -1;
  }

  uint8_t buff[2];
  ret = read_reg(LSM6DSV320X_CTRL1, buff, 2);
  ret += read_reg(LSM6DSV320X_HAODR_CFG, (uint8_t *)&haodr, 1);

  bytecpy((uint8_t *)&ctrl1, &buff[0]);
  bytecpy((uint8_t *)&ctrl2, &buff[1]);

  if (ret != 0)
    return ret;

  // cross-checking haodr mode
  uint8_t both_on = ctrl1.odr_xl != LSM6DSV320X_ODR_OFF &&
    ctrl2.odr_g != LSM6DSV320X_ODR_OFF ? 1 : 0;

  // if both on, then haodr_sel is a shared bit
  if (both_on && (gy_ha != haodr.haodr_sel))
  {
    return -1;
  }

  // if odr is choosed as an high-accuracy value, mode should be set in high-accuracy
  if ((gy_ha != 0 && gy_mode != LSM6DSV320X_GY_HIGH_ACCURACY_ODR_MD))
  {
    return -1;
  }

  // Switching (enable/disable) HAODR mode require that all sensors must be in power-down mode.
  // Note: lsm6dsv320x_haodr_set function should be called first.
  if (haodr.haodr_sel != gy_ha &&
      ctrl2.op_mode_g != gy_mode && // check if mode switch is required (prev. != new)
      (gy_mode == LSM6DSV320X_GY_HIGH_ACCURACY_ODR_MD || // check if mode to set is HAODR
       ctrl2.op_mode_g == LSM6DSV320X_GY_HIGH_ACCURACY_ODR_MD)) // check if previous mode was HAODR
  {
    ret += haodr_set(static_cast<lsm6dsv320x_data_rate_t>(ctrl1.odr_xl), 
    static_cast<lsm6dsv320x_xl_mode_t>(ctrl1.op_mode_xl),
     gy_odr, gy_mode);
  }
  else
  {
    // if HAODR switch is not required, just set ctrl2 settings
    ctrl2.op_mode_g = gy_mode;
    ctrl2.odr_g = gy_odr;
    haodr.haodr_sel = gy_ha;
    ret += write_reg(LSM6DSV320X_CTRL2, (uint8_t *)&ctrl2, 1);
    ret += write_reg(LSM6DSV320X_HAODR_CFG, (uint8_t *)&haodr, 1);
  }

  return ret;
}

/**
  * @brief HAODR set
  *        Allow changing the HAODR mode, which is a shared bit between the accelerometer and gyroscope.
  *        Both settings are required; this function should be used only if both sensors are already ON.
  *
  * @param  ctx        read / write interface definitions
  * @param  xl_odr     lsm6dsv320x_data_rate_t
  * @param  xl_mode    lsm6dsv320x_xl_mode_t
  * @param  gy_odr     lsm6dsv320x_data_rate_t
  * @param  gy_mode    lsm6dsv320x_gy_mode_t
  */
int32_t LSM6DSV320XClass::haodr_set(
  lsm6dsv320x_data_rate_t xl_odr,
  lsm6dsv320x_xl_mode_t xl_mode,
  lsm6dsv320x_data_rate_t gy_odr,
  lsm6dsv320x_gy_mode_t gy_mode)
{
  lsm6dsv320x_ctrl1_t ctrl1;
  lsm6dsv320x_ctrl2_t ctrl2;
  lsm6dsv320x_haodr_cfg_t haodr;
  lsm6dsv320x_ctrl1_xl_hg_t ctrl1_xl_hg;
  lsm6dsv320x_ctrl_eis_t ctrl_eis;
  lsm6dsv320x_ui_ctrl1_ois_t ctrl1_ois;
  int32_t ret;

  uint8_t xl_ha = (((uint8_t)xl_odr) >> 4) & 0xFU;
  uint8_t gy_ha = (((uint8_t)gy_odr) >> 4) & 0xFU;
  uint8_t both_on = xl_odr != LSM6DSV320X_ODR_OFF && gy_odr != LSM6DSV320X_ODR_OFF ? 1 : 0;

  if (both_on && (xl_ha != gy_ha))
  {
    return -1;
  }

  ret = read_reg(LSM6DSV320X_HAODR_CFG, (uint8_t *)&haodr, 1);
  ret += read_reg(LSM6DSV320X_CTRL1, (uint8_t *)&ctrl1, 1);
  ret += read_reg(LSM6DSV320X_CTRL2, (uint8_t *)&ctrl2, 1);
  ret += read_reg(LSM6DSV320X_CTRL1_XL_HG, (uint8_t *)&ctrl1_xl_hg, 1);
  ret += read_reg(LSM6DSV320X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);

  lsm6dsv320x_xl_mode_t prev_mode = static_cast<lsm6dsv320x_xl_mode_t>(ctrl1.op_mode_xl);
  lsm6dsv320x_ctrl1_xl_hg_t ctrl1_xl_hg_prev = ctrl1_xl_hg;
  lsm6dsv320x_ctrl_eis_t ctrl_eis_prev = ctrl_eis;

  if (ret != 0)
    return ret;

  // Enabling/disabling HAODR mode require to have all sensors in power-down mode
  ctrl1.odr_xl = LSM6DSV320X_ODR_OFF;
  ctrl2.odr_g = LSM6DSV320X_ODR_OFF;
  ctrl1_xl_hg.odr_xl_hg = LSM6DSV320X_HG_XL_ODR_OFF;
  ctrl1_xl_hg.xl_hg_regout_en = 0;
  ctrl_eis.odr_g_eis = LSM6DSV320X_EIS_ODR_OFF;
  ret = write_reg(LSM6DSV320X_CTRL1, (uint8_t *)&ctrl1, 1);
  ret += write_reg(LSM6DSV320X_CTRL2, (uint8_t *)&ctrl2, 1);
  // avoid turning off if already off
  if (ctrl1_xl_hg_prev.odr_xl_hg != LSM6DSV320X_HG_XL_ODR_OFF)
  {
    ret += write_reg(LSM6DSV320X_CTRL1_XL_HG, (uint8_t *)&ctrl1_xl_hg, 1);
  }
  if (ctrl_eis_prev.odr_g_eis != LSM6DSV320X_EIS_ODR_OFF)
  {
    ret += write_reg(LSM6DSV320X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);
  }

  // set HAODR
  haodr.haodr_sel = xl_ha | gy_ha;
  ctrl1.op_mode_xl = xl_mode;
  ctrl2.op_mode_g = gy_mode;

  ret += write_reg(LSM6DSV320X_HAODR_CFG, (uint8_t *)&haodr, 1);
  ret += write_reg(LSM6DSV320X_CTRL1, (uint8_t *)&ctrl1, 1);
  ret += write_reg(LSM6DSV320X_CTRL2, (uint8_t *)&ctrl2, 1);

  if (prev_mode == LSM6DSV320X_XL_HIGH_ACCURACY_ODR_MD)
  {
    delay(1);// should be at least 500 us; AN6119, section 3.4
  }

  // set xl and gy data rates and restore high-g xl and eis to their previous data rates
  ctrl1.odr_xl = xl_odr;
  ctrl2.odr_g = gy_odr;
  ret += write_reg(LSM6DSV320X_CTRL1, (uint8_t *)&ctrl1, 1);
  ret += write_reg(LSM6DSV320X_CTRL2, (uint8_t *)&ctrl2, 1);
  // if off, there is no need to turn them on
  if (ctrl1_xl_hg_prev.odr_xl_hg != LSM6DSV320X_HG_XL_ODR_OFF)
  {
    ret += write_reg(LSM6DSV320X_CTRL1_XL_HG, (uint8_t *)&ctrl1_xl_hg_prev, 1);
  }
  if (ctrl_eis_prev.odr_g_eis != LSM6DSV320X_EIS_ODR_OFF)
  {
    ret += write_reg(LSM6DSV320X_CTRL_EIS, (uint8_t *)&ctrl_eis_prev, 1);
  }

  return ret;

}

/**
  * @brief  Accelerometer HG full-scale selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      lsm6dsv320x_hg_xl_full_scale_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LSM6DSV320XClass::hg_xl_full_scale_set(lsm6dsv320x_hg_xl_full_scale_t val)
{
  lsm6dsv320x_ctrl1_xl_hg_t ctrl1;
  int32_t ret;

  ret = read_reg(LSM6DSV320X_CTRL1_XL_HG, (uint8_t *)&ctrl1, 1);


  if (ret == 0)
  {
    ctrl1.fs_xl_hg = (uint8_t)val & 0x7U;
  ret = write_reg(LSM6DSV320X_CTRL1_XL_HG, (uint8_t *)&ctrl1, 1);
  
}

  return ret;
}

/**
  * @brief  Accelerometer HG full-scale selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      lsm6dsv320x_hg_xl_full_scale_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LSM6DSV320XClass::hg_xl_full_scale_get(lsm6dsv320x_hg_xl_full_scale_t *val)
{
  lsm6dsv320x_ctrl1_xl_hg_t ctrl1;
  int32_t ret;

  ret = read_reg(LSM6DSV320X_CTRL1_XL_HG, (uint8_t *)&ctrl1, 1);
  
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl1.fs_xl_hg)
  {
    case LSM6DSV320X_32g:
      *val = LSM6DSV320X_32g;
      break;

    case LSM6DSV320X_64g:
      *val = LSM6DSV320X_64g;
      break;

    case LSM6DSV320X_128g:
      *val = LSM6DSV320X_128g;
      break;

    case LSM6DSV320X_256g:
      *val = LSM6DSV320X_256g;
      break;

    case LSM6DSV320X_320g:
      *val = LSM6DSV320X_320g;
      break;

    default:
      *val = LSM6DSV320X_32g;
      break;
  }

  return ret;
}

/**
  * @brief  Gyroscope full-scale selection[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      250dps, 500dps, 1000dps, 2000dps, 4000dps,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LSM6DSV320XClass::gy_full_scale_set(lsm6dsv320x_gy_full_scale_t val)
{
  lsm6dsv320x_ctrl6_t ctrl6;
  int32_t ret;

  ret = read_reg(LSM6DSV320X_CTRL6, (uint8_t *)&ctrl6, 1);


  if (ret == 0)
  {
    ctrl6.fs_g = (uint8_t)val & 0xfu;
    ret = write_reg(LSM6DSV320X_CTRL6, (uint8_t *)&ctrl6, 1);
  }


  return ret;
}

/**
  * @brief  Gyroscope full-scale selection[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      250dps, 500dps, 1000dps, 2000dps, 4000dps,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LSM6DSV320XClass::gy_full_scale_get(lsm6dsv320x_gy_full_scale_t *val)
{
  lsm6dsv320x_ctrl6_t ctrl6;
  int32_t ret;

  ret = read_reg(LSM6DSV320X_CTRL6, (uint8_t *)&ctrl6, 1);
  if (ret != 0)

  {
    return ret;
  }

  switch (ctrl6.fs_g)
  {
    case LSM6DSV320X_250dps:
      *val = LSM6DSV320X_250dps;
      break;

    case LSM6DSV320X_500dps:
      *val = LSM6DSV320X_500dps;
      break;

    case LSM6DSV320X_1000dps:
      *val = LSM6DSV320X_1000dps;
      break;

    case LSM6DSV320X_2000dps:
      *val = LSM6DSV320X_2000dps;
      break;

    case LSM6DSV320X_4000dps:
      *val = LSM6DSV320X_4000dps;
      break;

    default:
      *val = LSM6DSV320X_250dps;
      break;
  }

  return ret;
}

int32_t LSM6DSV320XClass::xl_full_scale_set(lsm6dsv320x_xl_full_scale_t val)
{
  lsm6dsv320x_ctrl8_t ctrl8;
  int32_t ret;

  ret = read_reg(LSM6DSV320X_CTRL8, (uint8_t *)&ctrl8, 1);
  //ret = lsm6dsv320x_read_reg(ctx, LSM6DSV320X_CTRL8, (uint8_t *)&ctrl8, 1);

  if (ret == 0)
  {
    ctrl8.fs_xl = (uint8_t)val & 0x3U;
    ret = write_reg(LSM6DSV320X_CTRL8, (uint8_t *)&ctrl8, 1);
    //ret = lsm6dsv320x_write_reg(ctx, LSM6DSV320X_CTRL8, (uint8_t *)&ctrl8, 1);
  }

  return ret;
}

int32_t LSM6DSV320XClass::xl_full_scale_get(lsm6dsv320x_xl_full_scale_t *val) {
  lsm6dsv320x_ctrl8_t ctrl8;
  int32_t ret;

  ret = read_reg(LSM6DSV320X_CTRL8, (uint8_t *)&ctrl8, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl8.fs_xl)
  {
    case LSM6DSV320X_2g:
      *val = LSM6DSV320X_2g;
      break;

    case LSM6DSV320X_4g:
      *val = LSM6DSV320X_4g;
      break;

    case LSM6DSV320X_8g:
      *val = LSM6DSV320X_8g;
      break;

    case LSM6DSV320X_16g:
      *val = LSM6DSV320X_16g;
      break;

    default:
      *val = LSM6DSV320X_2g;
      break;
  }

  return ret;
}

int32_t LSM6DSV320XClass::filt_settling_mask_set(bool mask_drdy, bool mask_irq_xl, bool mask_irq_g){
    lsm6dsv320x_filt_settling_mask_t filt_settling_mask{};
    filt_settling_mask.drdy = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g = PROPERTY_ENABLE;
    return _set_filter_settling_mask(filt_settling_mask);
}

int32_t LSM6DSV320XClass::_set_filter_settling_mask(lsm6dsv320x_filt_settling_mask_t val){
  lsm6dsv320x_emb_func_cfg_t emb_func_cfg;
  lsm6dsv320x_ui_int_ois_t ui_int_ois;
  lsm6dsv320x_ctrl4_t ctrl4;
  int32_t ret;

  ret = read_reg(LSM6DSV320X_CTRL4, (uint8_t *)&ctrl4, 1);
  if (ret != 0)
  {
    return ret;
  }
  ctrl4.drdy_mask = val.drdy;
  ret += write_reg(LSM6DSV320X_CTRL4, (uint8_t *)&ctrl4, 1);
  if (ret != 0)
  {
    return ret;
  }

  ret = read_reg(LSM6DSV320X_EMB_FUNC_CFG, (uint8_t *)&emb_func_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }
  emb_func_cfg.emb_func_irq_mask_xl_settl = val.irq_xl;
  emb_func_cfg.emb_func_irq_mask_xl_hg_settl = val.irq_xl_hg;
  emb_func_cfg.emb_func_irq_mask_g_settl = val.irq_g;
  ret += write_reg(LSM6DSV320X_EMB_FUNC_CFG, (uint8_t *)&emb_func_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  ret = read_reg(LSM6DSV320X_UI_INT_OIS, (uint8_t *)&ui_int_ois, 1);
  if (ret != 0)
  {
    return ret;
  }
  ui_int_ois.drdy_mask_ois = val.ois_drdy;
  ret += write_reg(LSM6DSV320X_UI_INT_OIS, (uint8_t *)&ui_int_ois, 1);

  return ret;
}

int32_t LSM6DSV320XClass::filt_gy_lp1_set(uint8_t val){
  lsm6dsv320x_ctrl7_t ctrl7;
  int32_t ret;

  ret = read_reg(LSM6DSV320X_CTRL7, (uint8_t *)&ctrl7, 1);
  if (ret == 0)
  {
    ctrl7.lpf1_g_en = val;
    ret = write_reg(LSM6DSV320X_CTRL7, (uint8_t *)&ctrl7, 1);
  }

  return ret;
}

int32_t LSM6DSV320XClass::filt_gy_lp1_bandwidth_set(lsm6dsv320x_filt_gy_lp1_bandwidth_t val)
{
  lsm6dsv320x_ctrl6_t ctrl6;
  int32_t ret;

  ret = read_reg(LSM6DSV320X_CTRL6, (uint8_t *)&ctrl6, 1);
  if (ret == 0)
  {
    ctrl6.lpf1_g_bw = (uint8_t)val & 0x0Fu;
    ret = write_reg(LSM6DSV320X_CTRL6, (uint8_t *)&ctrl6, 1);
  }

  return ret;
}

int32_t LSM6DSV320XClass::filt_xl_lp2_set(uint8_t val)
{
  lsm6dsv320x_ctrl9_t ctrl9;
  int32_t ret;

  ret = read_reg(LSM6DSV320X_CTRL9, (uint8_t *)&ctrl9, 1);
  if (ret == 0)
  {
    ctrl9.lpf2_xl_en = val;
    ret = write_reg(LSM6DSV320X_CTRL9, (uint8_t *)&ctrl9, 1);
  }

  return ret;
}

int32_t LSM6DSV320XClass::filt_xl_lp2_bandwidth_set(lsm6dsv320x_filt_xl_lp2_bandwidth_t val){
  lsm6dsv320x_ctrl8_t ctrl8;
  int32_t ret;

  ret = LSM6DSV320XClass::read_reg(LSM6DSV320X_CTRL8, (uint8_t *)&ctrl8, 1);
  if (ret == 0)
  {
    ctrl8.hp_lpf2_xl_bw = (uint8_t)val & 0x07U;
    ret = LSM6DSV320XClass::write_reg(LSM6DSV320X_CTRL8, (uint8_t *)&ctrl8, 1);
  }

  return ret;
}

int32_t LSM6DSV320XClass::sflp_enable_set(uint8_t val)
{
  lsm6dsv320x_emb_func_en_a_t emb_func_en_a;
  int32_t ret;

  ret = mem_bank_set(LSM6DSV320X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = read_reg(LSM6DSV320X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);
  if (ret != 0)
  {
    goto exit;
  }

  emb_func_en_a.sflp_game_en = val;
  ret += write_reg(LSM6DSV320X_EMB_FUNC_EN_A,
                               (uint8_t *)&emb_func_en_a, 1);

exit:
  ret += mem_bank_set(LSM6DSV320X_MAIN_MEM_BANK);

  return ret;
}

int32_t LSM6DSV320XClass::sflp_gravity_raw_get(int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret = mem_bank_set(LSM6DSV320X_EMBED_FUNC_MEM_BANK);
  ret += read_reg(LSM6DSV320X_SFLP_GRAVX_L, &buff[0], 6);
  ret += mem_bank_set(LSM6DSV320X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}

int32_t LSM6DSV320XClass::lsm6dsv320x_sflp_quaternion_raw_get(uint16_t *val)
{
  uint8_t buff[8];
  int32_t ret;

  ret = mem_bank_set(LSM6DSV320X_EMBED_FUNC_MEM_BANK);
  ret += read_reg(LSM6DSV320X_SFLP_QUATW_L, &buff[0], 8);
  ret += mem_bank_set(LSM6DSV320X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (uint16_t)buff[1];
  val[0] = (val[0] * 256) + (uint16_t)buff[0];
  val[1] = (uint16_t)buff[3];
  val[1] = (val[1] * 256) + (uint16_t)buff[2];
  val[2] = (uint16_t)buff[5];
  val[2] = (val[2] * 256) + (uint16_t)buff[4];
  val[3] = (uint16_t)buff[7];
  val[3] = (val[3] * 256) + (uint16_t)buff[6];

  return ret;
}

int32_t LSM6DSV320XClass::sflp_gbias_raw_get(int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret = mem_bank_set(LSM6DSV320X_EMBED_FUNC_MEM_BANK);
  ret += read_reg(LSM6DSV320X_SFLP_GBIASX_L, &buff[0], 6);
  ret += mem_bank_set(LSM6DSV320X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}

int32_t LSM6DSV320XClass::mem_bank_set(lsm6dsv320x_mem_bank_t val)
{
  lsm6dsv320x_func_cfg_access_t func_cfg_access;
  int32_t ret;

  ret = read_reg(LSM6DSV320X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  if (ret != 0)
  {
    return ret;
  }

  func_cfg_access.shub_reg_access = ((uint8_t)val & 0x02U) >> 1;
  func_cfg_access.emb_func_reg_access = (uint8_t)val & 0x01U;
  ret = write_reg(LSM6DSV320X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);

  return ret;
}

int32_t LSM6DSV320XClass::hg_xl_data_rate_set(lsm6dsv320x_hg_xl_data_rate_t val, uint8_t reg_out_en)
{
  lsm6dsv320x_ctrl1_t ctrl1;
  lsm6dsv320x_ctrl2_t ctrl2;
  lsm6dsv320x_ctrl1_xl_hg_t ctrl1_xl_hg;
  int32_t ret;

  ret = read_reg(LSM6DSV320X_CTRL1, (uint8_t *)&ctrl1, 1);
  ret += read_reg(LSM6DSV320X_CTRL2, (uint8_t *)&ctrl2, 1);
  ret += read_reg(LSM6DSV320X_CTRL1_XL_HG, (uint8_t *)&ctrl1_xl_hg, 1);
  if (ret != 0)
  {
    goto exit;
  }

  if (val != LSM6DSV320X_HG_XL_ODR_OFF && ctrl1.odr_xl != LSM6DSV320X_ODR_OFF &&
      ctrl1.op_mode_xl != LSM6DSV320X_XL_HIGH_PERFORMANCE_MD &&
      ctrl1.op_mode_xl != LSM6DSV320X_XL_HIGH_ACCURACY_ODR_MD)
  {
    ret = -1;
    goto exit;
  }

  // if xl or gy are ON in odr triggered mode, high-g xl cannot be turned on
  if ((ctrl1.odr_xl != LSM6DSV320X_ODR_OFF &&
       ctrl1.op_mode_xl == LSM6DSV320X_XL_ODR_TRIGGERED_MD) ||
      (ctrl2.odr_g != LSM6DSV320X_ODR_OFF &&
       ctrl2.op_mode_g == LSM6DSV320X_GY_ODR_TRIGGERED_MD))
  {
    ret = -1;
    goto exit;
  }

  ctrl1_xl_hg.odr_xl_hg = (uint8_t)val & 0x07U;
  ctrl1_xl_hg.xl_hg_regout_en = reg_out_en & 0x1U;
  ret += write_reg(LSM6DSV320X_CTRL1_XL_HG, (uint8_t *)&ctrl1_xl_hg, 1);

exit:
  return ret;
}