#include <FS.h>
#include <SPI.h>
#include <SD_MMC.h>

#include "SDLog.h"

/**
 * @brief Initializes the SD card logger
 * In MIDAS v2.x, the flash module has the same interface as
 * the SD card, we just need to set the appropiate pins
 * 
 * @return Error Code
*/
ErrorCode SDSink::init() {
    Serial.println("Connecting to SD...");
    if (!SD_MMC.setPins(FLASH_CLK, FLASH_CMD, FLASH_DAT0)) {
        return ErrorCode::SDBeginFailed;
    }
    if (!SD_MMC.begin("/sd", true, false, SDMMC_FREQ_52M, 5)) {
        failed = true;
        return ErrorCode::SDBeginFailed;
    }

    char file_name[16] = "data";
    char meta_name[24];
    char dfext[] = ".launch";
    sdFileNamer(file_name, ext, SD_MMC);

    strcpy(meta_name, file_name);
    char* extpos = strrchr(meta_name, '.');
    if (extpos) {
        strcpy(extpos, ".meta");
    }

    file = SD_MMC.open(file_name, FILE_WRITE, true);
    meta = SD_MMC.open(meta_name, FILE_WRITE, true);
    if (!file || !meta) {
        failed = true;
        return ErrorCode::SDCouldNotOpenFile;
    }

    return ErrorCode::NoError;
}

/**
 * @brief Writes a byte buffer to the SD card
 * 
 * @param data byte buffer of data
 * @param size size of buffer 
*/
void SDSink::write(const uint8_t* data, size_t size) {
    if (failed) {

    } else {
        file.write(data, size);
        unflushed_bytes += size;
        if(unflushed_bytes > 32768){
//        Serial.println("Flushed");
            file.flush();
            unflushed_bytes = 0;
        }
    }
}

void SDSink::write_meta(const uint8_t* data, size_t size) {
    if (failed) { return; }

    file.write(data, size);
    file.write('\n');
    file.flush(); // Meta writes are infrequent, so flushing is OK.
}