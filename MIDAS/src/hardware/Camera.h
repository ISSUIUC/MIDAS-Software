
static unsigned gSendDate = 0;
static unsigned gSentCount = 0;
static unsigned gReceivedCount = 0;

#define MCP2517_SCK 0
#define MCP2517_MOSI 0
#define MCP2517_MISO 0

#define MCP2517_CS 0
#define MCP2517_INT 255
#define CAN_bit_rate 125000 // CAN bit rate 125 kb/s

#define message_id 0x542 