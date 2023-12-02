#include "SDLog.h"
#include <FS.h>
#include <SD.h>

#define MAX_FILES 999

char* sdFileNamer(char* fileName, char* fileExtensionParam) {
    char fileExtension[strlen(fileExtensionParam) + 1];
    strcpy(fileExtension, fileExtensionParam);

    char inputName[strlen(fileName) + 1];
    strcpy(inputName, fileName);

    strcat(fileName, fileExtension);

    // checks to see if file already exists and adds 1 to filename if it does.
    if (SD.exists(fileName)) {
        bool fileExists = false;
        int i = 1;
        while (!fileExists) {
            if (i > MAX_FILES) {
                // max number of files reached. Don't want to overflow
                // fileName[]. Will write new data to already existing
                // data999.csv
                strcpy(fileName, inputName);
                strcat(fileName, "999");
                strcat(fileName, fileExtension);
                break;
            }

            // converts int i to char[]
            char iStr[16];
            itoa(i, iStr, 10);

            // writes "(sensor)_data(number).csv to fileNameTemp"
            char fileNameTemp[strlen(inputName) + strlen(iStr) + 6];
            strcpy(fileNameTemp, inputName);
            strcat(fileNameTemp, iStr);
            strcat(fileNameTemp, fileExtension);

            if (!SD.exists(fileNameTemp)) {
                strcpy(fileName, fileNameTemp);
                fileExists = true;
            }

            i++;
        }
    }

    return fileName;
}

ErrorCode LogSink::init() {
    if (SD.begin(0)) {  // todo pin
        char file_extension[8] = ".launch";

        char data_name[16] = "data";
        sdFileNamer(data_name, file_extension);
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

void LogSink::write(const uint8_t* data, size_t size) {
    file.write(data, size);
}
