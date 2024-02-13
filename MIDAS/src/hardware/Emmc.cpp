#include "Emmc.h"


void EMMCSink::write(const uint8_t* data, size_t size) {
    // file.print(message);
    // Get all data and write
    file.write(data, size);
}

ErrorCode EMMCSink::init(){
    if(!SD_MMC.setPins(EMMC_CLK, EMMC_CMD, EMMC_D0, EMMC_D1, EMMC_D2, EMMC_D3)){
        return ErrorCode::EmmcPinsAreWrong;
    }
    // if(!SD_MMC.begin()){
    if(!SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_52M, 5)){
        return ErrorCode::EmmcCouldNotBegin;
    }
    char file_extension[8] = ".launch";

    char data_name[16] = "data";
    sdFileNamer(data_name, file_extension, SD_MMC);

    file = SD_MMC.open(data_name, FILE_WRITE);

    if (!file) {
        return ErrorCode::EmmcCouldNotOpenFile;
    }
    
    return ErrorCode::NoError;
};