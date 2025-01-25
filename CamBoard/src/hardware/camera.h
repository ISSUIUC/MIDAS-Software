#pragma once
#include <HardwareSerial.h>
#include "errors.h"


struct Cameras {
    ErrorCode init();
    HardwareSerial cam1;
    HardwareSerial cam2;
};