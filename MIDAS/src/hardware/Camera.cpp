#include <ACAN2517.h>
#include <SPI.h>
#include "Camera.h"


static bool debug = false;

ACAN2517 can(MCP2517_CS, SPI, MCP2517_INT) ;

void can_setup () {
  ACAN2517Settings settings(ACAN2517Settings::OSC_4MHz10xPLL, CAN_bit_rate) ; // CAN bit rate 125 kb/s

  if (debug) {
    settings.mRequestedMode = ACAN2517Settings::InternalLoopBack ; // Select loopback mode
  }

  const uint32_t errorCode = can.begin(settings, [] { can.isr() ; }) ;
  if (errorCode != 0) {
    Serial.print("Configuration error 0x") ;
    Serial.println(errorCode, HEX) ;
  }
}

void can_loop (bool should_turn_cam_on) {
  CANMessage message ;
  if (gSendDate < millis()) {
    message.id = message_id ;
    message.data[0] = should_turn_cam_on;
    const bool ok = can.tryToSend(message) ;
    if (ok) {
      gSendDate += 2000 ;
    }
  }
  
  if (debug && can.receive (message)) {
    gReceivedCount += 1 ;
  }
}