#include<Arduino.h>
#include"E22.h"
#include"Command.h"
#include"Queue.h"
#include "midas_shell.h"
#include "esp_eeprom.h"



struct RadioConfig
{
    SX1268 * radio;
    Queue<TelemetryCommand>* cmd_queue;
    uint32_t frequency;
    uint32_t desired_frequency;
    uint8_t serial;
    uint8_t desired_serial;
    uint8_t indicator_led;
};

struct DuoSystems
{
    RadioConfig cfg0;
    RadioConfig cfg1;
    EEPROMController eeprom;
    MShell* shell;
};