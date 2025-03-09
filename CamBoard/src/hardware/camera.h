#pragma once
#include <HardwareSerial.h>
#include "errors.h"

struct read_mem_cap_data_return {
    uint8_t buf[32];
    uint8_t status;
  };

struct Cameras {
    ErrorCode init();
    HardwareSerial* cam1;
    HardwareSerial* cam2;
};


void camera_on_off(HardwareSerial& camera);

void start_recording(HardwareSerial& camera);

void stop_recording(HardwareSerial& camera);

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a);

uint8_t generate_crc(uint8_t* buf, unsigned int buf_len);

bool check_crc(uint8_t* buf, unsigned int buf_len, uint8_t expected_crc);

struct read_mem_cap_data_return read_mem_cap_data(HardwareSerial& camera);