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

    Serial.println("eMMC Connected");

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
    String message = Serial.readStringUntil('\n');

    if (message == "ls\n") {
        File root = fs.open("/");
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

        File file = root.openNextFile();
        while (file) {
            Serial.println(file.name());
            file = root.openNextFile();
        }
        Serial.println("<done>");
    } else if (message == "dump\n") {
        String file_name = Serial.readStringUntil('\n');
        file_name.trim();

        File file = fs.open("/" + file_name);
        if (!file) {
            Serial.println("Failure");
            Serial.println("<done>");
            return;
        }

        Serial.println("Success");

        uint32_t size = file.size();
        Serial.write((size) & 0xFF);
        Serial.write((size >> 8) & 0xFF);
        Serial.write((size >> 16) & 0xFF);
        Serial.write((size >> 24) & 0xFF);

        while (size) {
            file.read(buf, 1024);
            Serial.write(buf, 1024);
            if (size >= 1024) {
                size -= 1024;
            } else {
                size = 0;
            }
        }
        Serial.println("<done>");
    } else {
        Serial.println("Unrecognized command");
        Serial.println("<done>");
    }
}