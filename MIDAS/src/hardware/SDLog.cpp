#include <FS.h>
#include <SPI.h>
#include <SD_MMC.h>

#include "SDLog.h"
#include "pins.h"

/**
 * @brief Initializes the SD card logging system.
 *
 * Configures the SD/MMC interface, opens the flight log and metadata
 * files, and determines the next available log file name.
 *
 * On MIDAS V2.x, the onboard eMMC flash module is accessed through the
 * SD/MMC interface, so initialization is identical to an SD card after
 * assigning the appropriate pins.
 *
 * @return ErrorCode::NoError if initialization succeeds.
 * @return ErrorCode::SDBeginFailed if the SD/MMC interface or filesystem
 *         cannot be initialized.
 * @return ErrorCode::SDCouldNotOpenFile if the log files cannot be opened.
 */
ErrorCode SDSink::init() {

    // Configure the SD/MMC peripheral pins.
    Serial.println("[SD] Connecting to SD...");
    if (!SD_MMC.setPins(FLASH_CLK, FLASH_CMD, FLASH_DAT0, FLASH_DAT1, FLASH_DAT2, FLASH_DAT3)) {
        return ErrorCode::SDBeginFailed;
    }

    // Mount the filesystem.
    if (!SD_MMC.begin("/sd", true, false, SDMMC_FREQ_52M, 5)) {
        failed = true;
        return ErrorCode::SDBeginFailed;
    }

    Serial.println("[SD] Startup OK");

    // Determine the next available flight log filename.
    char file_name[16] = "data";
    char ext[] = ".bin";

    Serial.println("[SD] Determining output file");
    Serial.println(current_file_no);

    if(current_file_no != 0) {
        Serial.print("[SD] EEPROM log file recovered: ");
        Serial.println(current_file_no);
    }

    int filenumber = -1;
    sdFileNamer(file_name, ext, SD_MMC, current_file_no, &filenumber);

    if(filenumber != -1) {
        Serial.print("[SD] Beginning log: ");
        Serial.println(current_file_no);
    } else {
        failed = true;
        return ErrorCode::SDBeginFailed;
    }

    // Generate the metadata filename using the same base name.
    char meta_name[255] = {0};
    strcpy(meta_name, file_name);

    char* extpos = strrchr(meta_name, '.');
    if (extpos) {
        strcpy(extpos, ".meta");
    }

    // Open the primary log and metadata files.
    file = SD_MMC.open(file_name, FILE_WRITE, true);
    meta = SD_MMC.open(meta_name, FILE_WRITE, true);

    if (!file || !meta) {
        failed = true;
        return ErrorCode::SDCouldNotOpenFile;
    }

    // Store the allocated log number for future boots.
    current_file_no = static_cast<uint16_t>(filenumber);

    Serial.println("[SD] Init done");

    return ErrorCode::NoError;
}

/**
 * @brief Writes binary flight data to the log file.
 *
 * Data is buffered internally and periodically flushed to reduce the
 * number of expensive storage synchronization operations.
 *
 * @param data Pointer to the data buffer.
 * @param size Number of bytes to write.
 */
void SDSink::write(const uint8_t* data, size_t size) {

    // Write the data to the flight log.
    size_t bytes_written = file.write(data, size);

    // Track the amount of buffered data.
    unflushed_bytes += size;

    // Flush periodically to reduce data loss while minimizing write
    // overhead.
    if(unflushed_bytes > 32768) {
        file.flush();
        unflushed_bytes = 0;
    }

    // Record write failures.
    if(bytes_written != size) {
        failed_wr = true;
    }
}

/**
 * @brief Writes metadata to the metadata log file.
 *
 * Metadata entries are written infrequently, so the file is flushed after
 * every write to ensure the information is immediately committed.
 *
 * @param data Pointer to the metadata buffer.
 * @param size Number of bytes to write.
 */
void SDSink::write_meta(const uint8_t* data, size_t size) {

    // Skip writes if the storage subsystem has already failed.
    if (failed) {
        return;
    }

    // Write the metadata entry.
    size_t bytes_written = meta.write(data, size);

    // Separate metadata records with newlines.
    meta.write('\n');

    // Metadata writes are infrequent, so flushing immediately is
    // acceptable.
    meta.flush();

    // Record write failures.
    if(bytes_written != size) {
        failed_mr = true;
    }

    return;
}