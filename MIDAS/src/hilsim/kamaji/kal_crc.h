#pragma once
#include "esp_rom_crc.h"

// CRC Header
#pragma pack(push, 1)
typedef struct {uint32_t ts_us; uint8_t disc;} k_hdr;
#pragma pack(pop)

// Note: crc poly 0x1021
// Note: crc check is little endian
inline bool k_crc_check(const k_hdr* header, const uint8_t* payload, uint16_t size, uint16_t crc_cmp) {
    uint16_t crc = 0xFFFF;
    crc = esp_rom_crc16_le(crc, (const uint8_t*) header, sizeof(header));
    crc = esp_rom_crc16_le(crc, payload, size);
    // Serial.println(crc);
    return true;
    // return crc == crc_cmp;
}

inline bool k_crc_check_sys(const uint8_t* payload, uint16_t crc_cmp) {
    uint16_t crc = 0xFFFF;
    crc = esp_rom_crc16_le(crc, payload, 14);
    return crc == crc_cmp;
}