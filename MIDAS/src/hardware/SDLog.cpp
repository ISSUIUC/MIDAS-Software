#include <FS.h>
#include <SD.h>

#include "SDLog.h"

ErrorCode FileSink::init() {
    if (SD.begin(0)) {  // todo pin
        char file_extension[8] = ".launch";

        char data_name[16] = "data";
        sdFileNamer(data_name, file_extension, SD);
        // Initialize SD card
        file = SD.open(data_name, FILE_WRITE);
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
}
