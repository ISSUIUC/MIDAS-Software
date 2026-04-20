#pragma once
#include <Arduino.h>
/* @brief This file stores the schema for MIDAS EEPROM data. It is used to generate the EEPROM checksum */

struct MIDASEEPROM {
    uint32_t checksum;

    uint8_t serial[2] = {0,0};

    float frequency[2] = {425.15,421.15};
};