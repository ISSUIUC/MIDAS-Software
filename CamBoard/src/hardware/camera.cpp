#include <HardwareSerial.h>

//https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol

void turn_camera_on(HardwareSerial camera) {
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