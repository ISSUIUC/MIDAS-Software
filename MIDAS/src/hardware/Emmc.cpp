#include "Emmc.h"


void EMMCSink::write(const uint8_t* data, size_t size) {
    file.write(data, size);
    unflushed_bytes += size;
    if(unflushed_bytes > 32768){
        Serial.println("Flushed");
        file.flush();
        unflushed_bytes = 0;
    }
}

ErrorCode EMMCSink::init(){
    if(!SD_MMC.setPins(EMMC_CLK, EMMC_CMD, EMMC_D0, EMMC_D1, EMMC_D2, EMMC_D3)){
        return ErrorCode::EmmcPinsAreWrong;
    }
    // if(!SD_MMC.begin()){
    if(!SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_52M, 5)){
        return ErrorCode::EmmcCouldNotBegin;
    }
    char data_name[] = "/data.launch";

    file = SD_MMC.open(data_name, FILE_WRITE, true);

    if (!file) {
        return ErrorCode::EmmcCouldNotOpenFile;
    }
    
    return ErrorCode::NoError;
}
