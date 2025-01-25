#include <ACAN2517FD.h>
#include <ACAN2517FDSettings.h>
#include "sensors.h"
#include "CAN.h"


ACAN2517FD can (CAN_NCS, SPI, CAN_NINT);


ErrorCode CAN::init() {
    ACAN2517FDSettings can_settings (ACAN2517FDSettings::OSC_40MHz, 125*1000, ACAN2517FDSettings::DATA_BITRATE_x1  );
    can_settings.mRequestedMode = ACAN2517FDSettings::Normal20B;

    const uint32_t errorCode = can.begin (can_settings, [] { can.isr () ; }) ;
    if (0 == errorCode) {
        return ErrorCode::NoError;
    }else{
        Serial.print ("Error Can: 0x") ;
        Serial.println (errorCode, HEX) ;
        return ErrorCode::NoError; // Change this
    }
}


MIDASCommands CAN::read() {
}


bool CAN::recieve(CANFDMessage message) {
    return can.receive(message);
}