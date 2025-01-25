#include <HardwareSerial.h>
#include "errors.h"


struct Cameras {
    public:
    ErrorCode init();
    HardwareSerial cam1;
    HardwareSerial cam2;
};