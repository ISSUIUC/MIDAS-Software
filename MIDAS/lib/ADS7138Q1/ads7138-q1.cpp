#include "ads7138-q1.h"
#include<Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// see systems.cpp
extern SemaphoreHandle_t i2c_mutex;

#define WIRE Wire

#define ADC_ADDR 0x14
#define ADC_REG_READ 0x10
#define ADC_REG_WRITE 0x08
#define ADC_SET_BIT 0x18
#define ADC_CLEAR_BIT 0x20
#define ADC_REG_READ_CONTINUOUS 0x30
#define ADC_REG_WRITE_CONTINUOUS 0x28
#define ADC_SYSTEM_STATUS 0x00
#define ADC_GENERAL_CFG 0x01
#define ADC_OSR_CFG 0x03
#define ADC_OPMODE_CFG 0x04
#define ADC_MANUAL_CH_SEL 0x11
#define ADC_SEQUENCE_CFG 0x10
#define ADC_AUTO_SEQ_CH_SEL 0x12
#define ADC_CHANNEL_READ_START 0xA0

constexpr float kADCRegisterWidth = static_cast<float>(1 << 16);
constexpr float kADCVoltageReference = 3.3f;

bool ADS7138::init(TwoWire* i2c, uint8_t addr) {
  _i2c_addr = addr;
  _i2c = i2c;

  uint8_t res;

  if(!reg_read_single(ADC_SYSTEM_STATUS, res) || res == 0) {
    return false; // Failed to contact chip or failed to read
  }

  // Set sequencing channels
  uint8_t enable_byte = 0b11110111; // All channels but channel 3
  if(!reg_write_single(ADC_AUTO_SEQ_CH_SEL, enable_byte)) {
    return false;
  }

  // STATS_EN bit lets us read the RECENT_CH* channels
  if(!reg_write_single_bitset(ADC_GENERAL_CFG, 0b00100000)) {
    return false;
  }

  // CONV_MODE bit outputs values on internal clock
  if(!reg_write_single_bitset(ADC_OPMODE_CFG, 0b00100000)) {
    return false;
  }

  // Set OSR to 8 samples
  if(!reg_write_single_bitset(ADC_OPMODE_CFG, 0b00000011)) {
    return false;
  }

  // Enable auto sequencing
  enable_byte = 0b00010001; // Enable sequence, auto sequence
  if(!reg_write_single(ADC_SEQUENCE_CFG, enable_byte)) {
    return false;
  }

  return true;
}

void ADS7138::tick() {
  uint8_t buf[16];
  reg_read_block(ADC_CHANNEL_READ_START, 16, buf);

  for(int i = 0; i < 8; i++) {
    _ch_readings[i] = buf[i*2] + (buf[(i*2)+1] << 8);
  }
}

bool ADS7138::i2c_lock() {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    return true;
}

void ADS7138::i2c_unlock(void) {
    xSemaphoreGive(i2c_mutex);
}

bool ADS7138::reg_read_single(uint8_t addr, uint8_t& outval) {
    i2c_lock();
    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(ADC_REG_READ);
    _i2c->write(addr);
    if(WIRE.endTransmission(true) != 0){
      i2c_unlock();
      Serial.print("[ADS7138] Failed to read from addr 0x");
      Serial.println(addr, HEX);
      return false;
    }

    uint8_t ct = WIRE.requestFrom((int)_i2c_addr, 1, 1);
    if(ct != 1){
      i2c_unlock();
      Serial.print("[ADS7138] Failed to read from addr 0x");
      Serial.println(addr, HEX);
      return false;
    }

    outval = WIRE.read();
    i2c_unlock();
    return true;
}

bool ADS7138::reg_write_single_bitset(uint8_t reg_address, uint8_t set_mask) {
  i2c_lock();
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(ADC_SET_BIT);
  _i2c->write(reg_address);
  _i2c->write(set_mask);
  if(_i2c->endTransmission(true) != 0){
    i2c_unlock();
    Serial.print("[ADS7138] Failed to write bit mask to addr 0x");
    Serial.println(reg_address, HEX);
    return false;
  }
  i2c_unlock();
  return true;
}

bool ADS7138::reg_write_single(uint8_t reg_address, uint8_t data) {
  i2c_lock();
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(ADC_REG_WRITE);
  _i2c->write(reg_address);
  _i2c->write(data);
  if(_i2c->endTransmission(true) != 0){
    i2c_unlock();
    Serial.print("[ADS7138] Failed to write to addr 0x");
    Serial.println(reg_address, HEX);
    return false;
  }
  i2c_unlock();
  return true;
}

bool ADS7138::reg_read_block(uint8_t start_addr, size_t num_reg, uint8_t* out_buf) {
    i2c_lock();
    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(ADC_REG_READ_CONTINUOUS);
    _i2c->write(start_addr);
    if(WIRE.endTransmission(true) != 0){
      i2c_unlock();
      Serial.print("[ADS7138] Failed to read block from addr 0x");
      Serial.print(start_addr, HEX);
      Serial.print(" (Read size: ");
      Serial.print(num_reg);
      Serial.println(" B)");
      return false;
    }

    uint8_t ct = WIRE.requestFrom((int)_i2c_addr, (int) num_reg, 1);
    if(ct != num_reg){
      i2c_unlock();
      Serial.print("[ADS7138] Failed to read block from addr 0x");
      Serial.print(start_addr, HEX);
      Serial.print(" (Read size: ");
      Serial.print(num_reg);
      Serial.println(" B)");
      return false;
    }

    for(size_t i = 0; i < num_reg; i++) {
      out_buf[i] = WIRE.read();
    }

    i2c_unlock();
    return true;
}

float ADS7138::read(uint8_t pin) {
  uint16_t raw_value = _ch_readings[pin];

  float abs_voltage = ((static_cast<float>(raw_value) / kADCRegisterWidth) * kADCVoltageReference) / kADCPinConversionFactors[pin];
  return abs_voltage;
}