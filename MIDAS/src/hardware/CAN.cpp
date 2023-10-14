#include <ACAN.h>

static uint32_t gSendDate = 0 ;
static uint32_t gSentCount = 0 ;
static uint32_t gReceivedCount = 0 ;
static bool debug = true; // Allows loopback (only one device on bus)

void setup () {
  Serial.begin (9600) ;
  Serial.println ("Hello") ;
  ACANSettings settings (125 * 1000) ; // 125 kbit/s


  settings.mLoopBackMode = debug ;
  settings.mSelfReceptionMode = debug ;
  const uint32_t errorCode = ACAN::can0.begin(settings) ;
  if (0 == errorCode) {
    Serial.println ("can0 ok") ;
  }else{
    Serial.print ("Error can0: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

void loop () {
  CANMessage message ;
  if (gSendDate < millis ()) {
    message.id = 0x542 ;
    const bool ok = ACAN::can0.tryToSend (message) ;
    if (ok) {
      gSendDate += 2000 ;
      gSentCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentCount) ;
    }
  }

  if (ACAN::can0.receive (message)) {
    gReceivedCount += 1 ;
    Serial.print ("Received: ") ;
    Serial.println (gReceivedCount) ;
  }
}
