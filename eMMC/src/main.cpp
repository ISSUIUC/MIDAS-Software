#include <FS.h>
#include <SD_MMC.h>
#include <stdint.h>

#define EMMC_CLK 38
#define EMMC_CMD 39
#define EMMC_D0 44
#define EMMC_D1 43
#define EMMC_D2 2
#define EMMC_D3 42

uint8_t buffer[1024];

void setup() {
    Serial.begin(9600);

    while (!Serial) { }

    if (!SD_MMC.setPins(EMMC_CLK, EMMC_CMD, EMMC_D0, EMMC_D1, EMMC_D2, EMMC_D3)) {
        Serial.println("Pin change failed!");
        return;
    }
    // if(!SD_MMC.begin()){
    if (!SD_MMC.begin("/sdcard", false, true, SDMMC_FREQ_52M, 5)) {
        Serial.println("Card Mount Failed");
        return;
    }

    if (!SD_MMC.begin()) {
        Serial.println("Could not mount SDMMC.");
        return;
    }
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
            Serial.println(" bytes)");
            file = root.openNextFile();
        }
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
    } else {
        Serial.println("Unrecognized command: " + message);
        Serial.println("<done>");
    }
}