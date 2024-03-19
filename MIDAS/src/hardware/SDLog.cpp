#include <FS.h>
//#include <SD.h>
#include <SD_MMC.h>

#include "SDLog.h"


// todo none of this works


ErrorCode FileSink::init() {
    Serial.println("Connecting to SD...");
    if (!SD_MMC.setPins(5, 4, 6)) {
        return ErrorCode::SDBeginFailed;
    }
    if (SD_MMC.begin("/sd", true, true, 40000, 5)) {
//        char file_extension[8] = ".launch";

        char data_name[16] = "/data.launch";
//        sdFileNamer(data_name, file_extension, SD_MMC);
        // Initialize SD card
        file = SD_MMC.open(data_name, FILE_WRITE);
        // print header to file on sd card that lists each variable that is logged
        // sd_file.println("binary logging of sensor_data_t");
        file.flush();

        Serial.println(file.name());
    } else {
        return ErrorCode::SDBeginFailed;
    }
    return ErrorCode::NoError;
}

void FileSink::write(const uint8_t* data, size_t size) {
    file.write(data, size);
    file.flush();
}
