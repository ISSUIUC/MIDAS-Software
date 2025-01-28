#include <HardwareSerial.h>
#include "camera.h"
#include "pins.h"

//https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol

ErrorCode Cameras::init() {
    Serial.println("Starting UART...");
    //HardwareSerial cam1(1);
    cam1.begin(115200, SERIAL_8N1, CAM1_RX, CAM1_TX);
    //HardwareSerial cam2(2);
    cam2.begin(115200, SERIAL_8N1, CAM2_RX, CAM2_TX);
   
    return ErrorCode::NoError; // Change this
}

void camera_on_off(HardwareSerial camera) {
    uint8_t arr[4] = {0xCC, 0x01, 0x01, 0xE7};
    camera.write(arr, 4);
}

void start_recording(HardwareSerial camera) {
    uint8_t arr[4] = {0xCC, 0x01, 0x03, 0x98};
    camera.write(arr, 4);
}

void stop_recording(HardwareSerial camera) {
    uint8_t arr[4] = {0xCC, 0x01, 0x04, 0xCC};
    camera.write(arr, 4);
}