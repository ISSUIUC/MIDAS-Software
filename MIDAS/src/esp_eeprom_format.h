#pragma once
#include <sensor_data.h>
/* @brief This file stores the schema for MIDAS EEPROM data. It is used to generate the EEPROM checksum */

struct MIDASEEPROM {
    uint32_t checksum;

    uint16_t sd_file_num_last = 0;

    Acceleration lsm6dsv320x_hg_xl_bias = {0.0f, 0.0f, 0.0f};
    Magnetometer mmc5983ma_softiron_bias = {1.0f, 1.0f, 1.0f};
    Magnetometer mmc5983ma_hardiron_bias = {0.0f, 0.0f, 0.0f};
};