#include <FS.h>
#include <SPI.h>
#include <SD_MMC.h>

#include "SDLog.h"

#include "pins.h"

#define MAX_FILES 999

/**
 * @brief names a new file for a log sink depending on the files currently on said LogSink
 *
 * @param fileName buffer to write the file name to
 * @param fileExtensionParam the file extension required for the file
 * @param fs the FileSystem to check files off of
 *
 * @return buffer contianing string of file name
*/
char* sdFileNamer(char* fileName, char* fileExtensionParam, FS& fs) {
    char fileExtension[strlen(fileExtensionParam) + 1];
    strcpy(fileExtension, fileExtensionParam);

    char inputName[256] = {0};
    strcpy(inputName, "/");
    strcat(inputName, fileName);
    strcat(inputName, fileExtension);

    // checks to see if file already exists and adds 1 to filename if it does.
    bool exists = fs.exists(inputName);

    if (exists) {
        bool fileExists = false;
        int i = 1;
        while (!fileExists) {
            if (i > MAX_FILES) {
                // max number of files reached. Don't want to overflow
                // fileName[]. Will write new data to already existing
                // data999.csv
                strcpy(inputName, "/");
                strcat(inputName, fileName);
                strcat(inputName, "999");
                strcat(inputName, fileExtension);
                break;
            }

            // converts int i to char[]
            char iStr[16] = {0};
            itoa(i, iStr, 10);

            // writes "(sensor)_data(number).csv to fileNameTemp"
            strcpy(inputName, "/");
            strcat(inputName, fileName);
            strcat(inputName, iStr);
            strcat(inputName, fileExtension);

            if (!fs.exists(inputName)) {
                fileExists = true;
            }

            i++;
        }
    }

    strcpy(fileName, inputName);

    return fileName;
}

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

/**
 * @brief Writes a byte buffer to the emmc
 *
 * @param data byte buffer of data
 * @param size size of buffer
*/
void EMMCSink::write(const uint8_t* data, size_t size) {
    file.write(data, size);
    unflushed_bytes += size;
    if(unflushed_bytes > 32768){
        file.flush();
        unflushed_bytes = 0;
    }
}

/**
 * @brief Init the emmc
 *
 * @return Error code
*/
ErrorCode EMMCSink::init(){
    if(!SD_MMC.setPins(EMMC_CLK, EMMC_CMD, EMMC_D0, EMMC_D1, EMMC_D2, EMMC_D3)){
        return ErrorCode::EmmcPinsAreWrong;
    }
    // if(!SD_MMC.begin()){
    if(!SD_MMC.begin("/sdcard", false, false, SDMMC_FREQ_52M, 5)){
        return ErrorCode::EmmcCouldNotBegin;
    }
    char file_name[16] = "data";
    char ext[] = ".launch";
    sdFileNamer(file_name, ext, SD_MMC);

    file = SD_MMC.open(file_name, FILE_WRITE, true);

    if (!file) {
        return ErrorCode::EmmcCouldNotOpenFile;
    }

    return ErrorCode::NoError;
}
