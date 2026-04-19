#pragma once
#include <Arduino.h>
/* @brief This file stores the schema for MIDAS EEPROM data. It is used to generate the EEPROM checksum */

struct MIDASEEPROM {
    uint32_t checksum;

    uint8_t serial0 = 0;
    uint8_t serial1 = 0;

    float frequency0 = 425.15;
    float frequency1 = 421.15;
};