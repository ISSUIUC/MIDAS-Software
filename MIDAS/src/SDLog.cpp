#include <FS.h>
#include <SPI.h>
#include <SD_MMC.h>

#include "hardware/pins.h"
#include "SDLog.h"

/**
 * @brief Initializes the SD card logger
 * 
 * @return Error Code
*/
ErrorCode SDSink::init() {
    Serial.println("Connecting to SD...");
    if (!SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0)) {
        return ErrorCode::SDBeginFailed;
    }
    if (!SD_MMC.begin("/sd", true, false, SDMMC_FREQ_52M, 5)) {
        failed = true;
        return ErrorCode::NoError;
    }

    char file_name[16] = "data";
    char ext[] = ".launch";
    sdFileNamer(file_name, ext, SD_MMC);

    file = SD_MMC.open(file_name, FILE_WRITE, true);
    if (!file) {
        failed = true;
        return ErrorCode::NoError;
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
