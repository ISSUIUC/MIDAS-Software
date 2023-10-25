#include <ACAN2517.h>
#include <SPI.h>

static const byte MCP2517_SCK  = 0;
static const byte MCP2517_MOSI = 0;
static const byte MCP2517_MISO = 0;

static const byte MCP2517_CS  = 0;
static const byte MCP2517_INT = 0;
static bool debug = false;

ACAN2517 can (MCP2517_CS, SPI, MCP2517_INT) ;

void setup () {

  Serial.begin(9600);

  while (!Serial) {
    delay (50) ;
  }

  SPI.begin (MCP2517_SCK, MCP2517_MISO, MCP2517_MOSI) ;
  // Serial.print ("sizeof (ACAN2517Settings): ") ;
  // Serial.print (sizeof (ACAN2517Settings)) ;
  // Serial.println (" bytes") ;
  // Serial.println ("Configure ACAN2517") ;
  ACAN2517Settings settings (ACAN2517Settings::OSC_4MHz10xPLL, 125 * 1000) ; // CAN bit rate 125 kb/s

  if (debug) {
    settings.mRequestedMode = ACAN2517Settings::InternalLoopBack ; // Select loopback mode
  }

  const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  if (errorCode != 0) {
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

static unsigned gSendDate = 0 ;
static unsigned gSentCount = 0 ;
static unsigned gReceivedCount = 0 ;

void loop (bool should_turn_cam_on) {
  CANMessage message ;
  if (gSendDate < millis()) {
    message.id = 0x542 ;
    message.data[0] = should_turn_cam_on ? 1 : 0;
    const bool ok = can.tryToSend(message) ;
    if (ok) {
      gSendDate += 2000 ;
    }
  }
  
  if (debug && can.receive (message)) {
    gReceivedCount += 1 ;
    Serial.print ("Received: ") ;
    Serial.println (gReceivedCount) ;
  }
}