#if defined ESP8266 || defined ESP32
#include <SPI.h>
#include "boards/mcu/board.h"

SPIClass SPI_LORA;

void initSPI(void)
{
#ifdef ESP8266
#else
#endif
}
#endif