#include "hilsim.h"

#include <Arduino.h>
#include <pb_decode.h>

#include "hilsim/hilsimpacket.pb.h"

void hilsim() {
    char buffer[HILSIMPacket_size];
    // Read the thing
    Serial.readBytes(buffer, HILSIMPacket_size);
    
}
