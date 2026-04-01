#pragma once

// Handler for ESP32's eeprom read/writes for MIDAS FSW.
#include "esp_eeprom_format.h"
#include "esp_eeprom_checksum.h"
#include "errors.h"
#include <Arduino.h>
#include <EEPROM.h>


class EEPROMController {
    public:
    /* Reads the on-board EEPROM memory and updates `data`. Returns `true` if data was successfully read from EEPROM. Only updates `data` if successfully read. */
    bool read();

    /* Writes to the on-board EEPROM memory, then performs a `read_flash` operation. Returns true if the write and readback is successful. */
    bool commit();


    /* Will initialize the controller, and perform the first read. */
    ErrorCode init();

    /* The actual EEPROM data. Only updated by the driver if `read_flash()` is called (also after a `commit()`) */
    MIDASEEPROM data;
};