#pragma once
#include <sensor_data.h>
/* @brief This file stores the schema for MIDAS EEPROM data. It is used to generate the EEPROM checksum */

struct MIDASEEPROM {
    uint32_t checksum;

    Acceleration lsm6dsv320x_hg_xl_bias;
    Magnetometer mmc5983ma_softiron_bias;
    Magnetometer mmc5983ma_hardiron_bias;
};