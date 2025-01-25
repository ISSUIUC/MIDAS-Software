#include <ACAN2517FD.h>
#include <ACAN2517FDSettings.h>
#include "errors.h"
#include "sensor_data.h"

struct CAN {
    ErrorCode init();
    MIDASCommands read();
    bool recieve(CANFDMessage command);
};