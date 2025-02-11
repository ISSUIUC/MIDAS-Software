#pragma once
#include <HardwareSerial.h>
#include "errors.h"


struct Cameras {
    ErrorCode init();
    HardwareSerial cam1;
    HardwareSerial cam2;
};


void camera_on_off(HardwareSerial& camera);

void start_recording(HardwareSerial& camera);

void stop_recording(HardwareSerial& camera);