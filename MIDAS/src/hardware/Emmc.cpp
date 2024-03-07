#include "Emmc.h"


void EMMCSink::write(const uint8_t* data, size_t size) {
    // Get all data and write
    file.write(data, size);
    file.flush();
    Serial.print("Wrote ");
    Serial.print(size);
    Serial.println(" bytes");
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
