#include <cstdint>
#include <stddef.h>

#define CRC16 0x1021

uint16_t calculateCRC16(const uint8_t *data, size_t length) {
    uint16_t crc = CRC16; // Initial value for CRC-16/MODBUS
    uint16_t polynomial = 0xA001; // Reversed polynomial for CRC-16/MODBUS (0x8005 reversed)

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i]; // XOR current byte with CRC
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) { // If LSB is 1
                crc >>= 1; // Shift right
                crc ^= polynomial; // XOR with polynomial
            } else {
                crc >>= 1; // Shift right
            }
        }
    }
    return crc;
}