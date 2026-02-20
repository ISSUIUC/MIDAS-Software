#include <esp_eeprom.h>

constexpr size_t EEPROM_SIZE = sizeof(MIDASEEPROM);

bool EEPROMController::read() {
    MIDASEEPROM _read;
    uint8_t buf[EEPROM_SIZE];
    for (int i = 0; i < EEPROM_SIZE; i++) {
        buf[i] = EEPROM.read(i);
    }

    memcpy(&_read, buf, EEPROM_SIZE);

    if(_read.checksum != EEPROM_CHECKSUM) {
        // Wrong checksum, cannot read.
        return false;
    }

    // Otherwise, we set the data to the read EEPROM.
    data = _read;

    return true;
}

bool EEPROMController::commit() {
    uint8_t buf[EEPROM_SIZE];
    data.checksum = EEPROM_CHECKSUM;

    memcpy(buf, &data, EEPROM_SIZE);

    for (int i = 0; i < EEPROM_SIZE; i++) {
        EEPROM.write(i, buf[i]);
    }

    EEPROM.commit();

    return read();
}

ErrorCode EEPROMController::init() {

    EEPROM.begin((size_t)EEPROM_SIZE);

    if (!read()) {
        // The current eeprom format is incompatible, so we're going to default initialize MIDASEEPROM
        MIDASEEPROM empty_setting;
        empty_setting.checksum = EEPROM_CHECKSUM;
        data = empty_setting;
        Serial.println("EEPROM INITIAL READ FAILED");
        commit(); // should we do this? essentially wipes eeprom. probably doesn't matter though if format is incompatible
    }

    return ErrorCode::NoError;
}

