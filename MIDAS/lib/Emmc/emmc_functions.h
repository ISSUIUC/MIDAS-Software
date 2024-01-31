#pragma once

#include <FS.h>
#include <SD_MMC.h>

// void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
//     Serial.printf("Listing directory: %s\n", dirname);

//     File root = fs.open(dirname);
//     if(!root){
//         Serial.println("Failed to open directory");
//         return;
//     }
//     if(!root.isDirectory()){
//         Serial.println("Not a directory");
//         return;
//     }

//     File file = root.openNextFile();
//     while(file){
//         if(file.isDirectory()){
//             Serial.print("  DIR : ");
//             Serial.println(file.name());
//             if(levels){
//                 listDir(fs, file.path(), levels -1);
//             }
//         } else {
//             Serial.print("  FILE: ");
//             Serial.print(file.name());
//             Serial.print("  SIZE: ");
//             Serial.println(file.size());
//         }
//         file = root.openNextFile();
//     }
// }

// void createDir(fs::FS &fs, const char * path){
//     Serial.printf("Creating Dir: %s\n", path);
//     if(fs.mkdir(path)){
//         Serial.println("Dir created");
//     } else {
//         Serial.println("mkdir failed");
//     }
// }

// void removeDir(fs::FS &fs, const char * path){
//     Serial.printf("Removing Dir: %s\n", path);
//     if(fs.rmdir(path)){
//         Serial.println("Dir removed");
//     } else {
//         Serial.println("rmdir failed");
//     }
// }

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
}