#include <FS.h>
#include <SD_MMC.h>
#include <stdint.h>

#define FLASH_CMD 14
#define FLASH_CLK 18
#define FLASH_DAT0 13
#define FLASH_DAT1 12
#define FLASH_DAT2 15
#define FLASH_DAT3 16

#define EMMC_CLK 38
#define EMMC_CMD 39
#define EMMC_D0 44
#define EMMC_D1 43
#define EMMC_D2 2
#define EMMC_D3 42

uint8_t buffer[1024];

void setup() {
    Serial.begin(9600);

    delay(2000);

    while (!Serial) { }

    Serial.println("setup");

    Serial.println("Connecting to SD...");
    if (!SD_MMC.setPins(FLASH_CLK, FLASH_CMD, FLASH_DAT0)) {
        Serial.println("Pin change failed!");
        return;
    }
    if (!SD_MMC.begin("/sd", true, false, SDMMC_FREQ_52M, 5)) {
        Serial.println("Card Mount Failed");
        return;
    }

    // if (!SD_MMC.setPins(FLASH_CLK, FLASH_CMD, FLASH_DAT0)) {
    //     Serial.println("Pin change failed!");
    //     return;
    // }

    // // if(!SD_MMC.begin()){
    // if (!SD_MMC.begin("/sd", true, false, SDMMC_FREQ_52M, 5)) {
    //     Serial.println("Card Mount Failed");
    //     return;
    // }

    // if (!SD_MMC.begin()) {
    //     Serial.println("Could not mount SDMMC.");
    //     return;
    // }

    Serial.println("success!");
}


void loop() {
    String message;
    while (true) {
        message = Serial.readStringUntil('\n');
        message.trim();
        if (message != "") {
            break;
        }
    }

    if (message == "ls") {
        size_t tot_size = 0;
        File root = SD_MMC.open("/");
        if (!root){
            Serial.println("Failed to open directory");
            Serial.println("<done>");
            return;
        }
        if (!root.isDirectory()) {
            Serial.println("Not a directory");
            Serial.println("<done>");
            return;
        }

        Serial.println(" | name (size) ");
        Serial.println("-+-------------");
        File file = root.openNextFile();
        while (file) {
            Serial.print(" | ");
            Serial.print(file.name());
            Serial.print(" (");
            Serial.print(file.size());
            tot_size += file.size();
            Serial.println(" bytes)");
            file = root.openNextFile();
        }
        Serial.print("Total size taken up: ");
        Serial.print(tot_size);
        Serial.println("B / 8 000 000 000B");
        Serial.println("<done>");
    } else if (message == "dump") {
        String file_name = Serial.readStringUntil('\n');
        file_name.trim();

        File file = SD_MMC.open("/" + file_name);
        if (!file) {
            Serial.println("Could not open file.");
            Serial.println("<done>");
            return;
        }

        Serial.println("Success");

        uint32_t size = file.size();
        Serial.print("Dumping ");
        Serial.print(size);
        Serial.println(" bytes...");

        Serial.write((size) & 0xFF);
        Serial.write((size >> 8) & 0xFF);
        Serial.write((size >> 16) & 0xFF);
        Serial.write((size >> 24) & 0xFF);

        while (size) {
            file.read(buffer, 1024);
            if (size >= 1024) {
                Serial.write(buffer, 1024);
                size -= 1024;
            } else {
                Serial.write(buffer, size);
                size = 0;
            }
        }
        Serial.println("<done>");
    } else if (message == "<restart>") {
        Serial.println("eMMC Connected");
    } else if (message == "rmall") {
        // Delete all files in the root directory   
        File root = SD_MMC.open("/");
        while (true) {
            File file = root.openNextFile();
            if (!file) {
                break; // No more files
            }
            Serial.print("Deleting file: ");
            Serial.println(file.path());
            file.close();
            file.flush();
            SD_MMC.remove(file.path());  
        }
        Serial.println("All files deleted!");
        Serial.println("<done>");
    } 
    else {
        Serial.println("Unrecognized command: " + message);
        Serial.println("<done>");
    }
}