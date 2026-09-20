#include "esp_eeprom.h"

/**
 * @brief Reads and validates the contents of EEPROM.
 *
 * The raw EEPROM bytes are copied into a temporary MIDASEEPROM structure and
 * verified using the stored checksum. If the checksum does not match the
 * expected EEPROM schema checksum, the read is considered invalid and no data
 * is loaded.
 *
 * @return true if the EEPROM contents are valid and successfully loaded into
 *         the controller.
 * @return false if the checksum is invalid or the stored data is incompatible
 *         with the current EEPROM layout.
 */
bool EEPROMController::read() {
    MIDASEEPROM _read;
    uint8_t buf[EEPROM_SIZE];

    // Read the raw EEPROM contents into a temporary buffer.
    for (int i = 0; i < EEPROM_SIZE; i++) {
        buf[i] = EEPROM.read(i);
    }

    // Deserialize the buffer into the EEPROM schema.
    memcpy(&_read, buf, EEPROM_SIZE);

    // Verify the stored checksum before accepting the data.
    if(_read.checksum != EEPROM_CHECKSUM) {
        // Wrong checksum, cannot read.
        return false;
    }

    // Store the validated EEPROM contents.
    data = _read;

    return true;
}

/**
 * @brief Writes the current EEPROM data to non-volatile memory.
 *
 * The checksum is updated before serialization to ensure future reads can
 * verify data integrity. After writing and committing the data, the EEPROM is
 * immediately reread to confirm the stored contents are valid.
 *
 * @return true if the committed data can be successfully read back.
 * @return false if verification fails after committing.
 */
bool EEPROMController::commit() {
    uint8_t buf[EEPROM_SIZE];

    // Update the checksum before writing.
    data.checksum = EEPROM_CHECKSUM;

    // Serialize the EEPROM schema into a raw byte buffer.
    memcpy(buf, &data, EEPROM_SIZE);

    // Write each byte to EEPROM.
    for (int i = 0; i < EEPROM_SIZE; i++) {
        EEPROM.write(i, buf[i]);
    }

    // Commit pending writes to non-volatile memory.
    EEPROM.commit();

    // Verify the written data.
    return read();
}

/**
 * @brief Initializes the EEPROM controller.
 *
 * Allocates the EEPROM storage region and attempts to load the stored
 * configuration. If the stored checksum is invalid or the EEPROM layout is
 * incompatible with the current firmware, the EEPROM is reset to default
 * values and rewritten using the current schema.
 *
 * @return ErrorCode::NoError after initialization completes.
 */
ErrorCode EEPROMController::init() {
    // Ensure the configured EEPROM layout fits within the hardware limit.
    static_assert(EEPROM_SIZE <= EEPROM_MAX_SIZE);

    EEPROM.begin((size_t)EEPROM_SIZE);

    if (!read()) {
        // The stored EEPROM schema is incompatible with this firmware version.
        // Create a default EEPROM image with the correct checksum.
        MIDASEEPROM empty_setting;
        empty_setting.checksum = EEPROM_CHECKSUM;
        data = empty_setting;

        Serial.println("EEPROM CHECKSUM INCOMPATIBLE");

        // Rewrite EEPROM using the default configuration.
        commit();
    }

    return ErrorCode::NoError;
}