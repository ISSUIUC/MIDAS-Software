#include "Emmc.h"

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
    //process profiling
    return ErrorCode::NoError;
}
