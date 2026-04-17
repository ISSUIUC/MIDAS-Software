#include<Arduino.h>
#include<SPI.h>
#include"FreeRTOS.h"
#include"E22.h"
#include"duo_pins.h"
#include"Packet.h"
#include"Output.h"
#include"Command.h"
#include"Queue.h"

constexpr uint32_t RADIO0_FREQ = 425150000;
constexpr uint32_t RADIO1_FREQ = 421150000;
Queue<TelemetryCommand> radio0_cmds;
Queue<TelemetryCommand> radio1_cmds;

struct RadioConfig
{
    SX1268 * radio;
    Queue<TelemetryCommand>* cmd_queue;
    uint32_t frequency;
    uint8_t serial;
    uint8_t indicator_led;
};


bool init_radio(SX1268& radio, uint32_t frequency) {
    if(radio.setup() != SX1268Error::NoError) return false;
    if(radio.set_frequency(frequency) != SX1268Error::NoError) return false;
    if(radio.set_modulation_params(8, LORA_BW_250, LORA_CR_4_8, false) != SX1268Error::NoError) return false;
    if(radio.set_tx_power(22) != SX1268Error::NoError) return false;

    return true;
}

void Radio_Rx_Thread(void * arg) {
    RadioConfig* cfg = (RadioConfig*)arg;
    bool led_state = false;
    bool reset_state = false;
    bool initial_ack_flag = true;
    TelemetryCommand to_send = buildTelemCmd(CommandType::EMPTY, cfg->serial);

    while(true) {
        TelemetryPacket packet{};
        SX1268Error res = cfg->radio->recv((uint8_t*)&packet, sizeof(packet), -1);
        if(res == SX1268Error::NoError) {
            led_state = !led_state;
            digitalWrite(cfg->indicator_led, led_state);
            FullTelemetryData data = DecodePacket(packet, cfg->frequency / 1e6);
            data.rssi = cfg->radio->get_last_snr();
            printPacketJson(data);

            if(initial_ack_flag) {
                reset_state = data.cmd_ack;
                initial_ack_flag = false;
            }

            if(data.cmd_ack != reset_state) {
                reset_state = data.cmd_ack;
                Serial.println(json_command_ack);
                to_send.command = CommandType::EMPTY;
            }

            if(to_send.command == CommandType::EMPTY) {
                if(!cfg->cmd_queue->receive(&to_send)){
                    to_send.command = CommandType::EMPTY;
                }
            }

            if(to_send.command != CommandType::EMPTY) {
                (void)cfg->radio->send((uint8_t*)&to_send, sizeof(to_send));
                Serial.println(json_command_sent);
            }
        }
    }
}

// Identifies this device over serial
void serial_identify() {
    Serial.println("IDENT_RESPONSE:FEATHER_DUO");
}

void handle_serial(const String& key) {


    if (key == "IDENT") {
        serial_identify();
        return;
    }

    TelemetryCommand command{};

    if(key.length() < 1) {
        Serial.println(json_command_bad);
        return;
    }

    String cmd_name = key.substring(1);

    if (cmd_name == "RESET_KF") {
        command.command = CommandType::RESET_KF;
    } else if (cmd_name == "SAFE") {
        command.command = CommandType::SWITCH_TO_SAFE;
    } else if (cmd_name == "IDLE") {
        command.command = CommandType::SWITCH_TO_IDLE;
    } else if (cmd_name == "PT") {
        command.command = CommandType::SWITCH_TO_PYRO_TEST;
    } else if (cmd_name == "PA") {
        command.command = CommandType::FIRE_PYRO_A;
    } else if (cmd_name == "PB") {
        command.command = CommandType::FIRE_PYRO_B;
    } else if (cmd_name == "PC") {
        command.command = CommandType::FIRE_PYRO_C;
    } else if (cmd_name == "PD") {
        command.command = CommandType::FIRE_PYRO_D;
    } else if (cmd_name == "CAMON") {
        command.command = CommandType::CAM_ON;
    } else if (cmd_name == "CAMOFF") {
        command.command = CommandType::CAM_OFF;
    } else if (cmd_name == "VMUXT") {
        command.command = CommandType::TOGGLE_CAM_VMUX;
    } else if (cmd_name == "CXL") {
        command.command = CommandType::CALIB_ACCEL;
    } else if (cmd_name == "CMG") {
        command.command = CommandType::CALIB_MAG;
    } else {
        Serial.println(json_command_bad);
        return;
    }

    if(key[0] == 0) {
        radio0_cmds.send(command);
    }
    if(key[0] == 1) {
        radio1_cmds.send(command);
    }

    Serial.println(json_command_success);
}

void Management_Thread(void * arg) {
    String cur_input = "";
    bool led_state = false;
    while(true){
        while(Serial.available()) {
            char input = Serial.read();
            if(input == '\n') {
                cur_input.replace("\r", "");
                handle_serial(cur_input);
                cur_input = "";
            } else {
                cur_input += input;
            }
        }
        led_state = !led_state;
        digitalWrite(Pins::LED_GREEN, led_state);
        delay(10); 
    }
}


void setup() {
    Serial.begin(460800);
    
    SPIClass SPI0(HSPI);
    SPIClass SPI1(FSPI);
    SX1268 Radio0(SPI0, Pins::E22_CS_0, Pins::E22_BUSY_0, Pins::E22_DIO1_0, Pins::E22_RXEN_0, Pins::E22_TXEN_0, Pins::E22_RESET_0);
    SX1268 Radio1(SPI1, Pins::E22_CS_1, Pins::E22_BUSY_1, Pins::E22_DIO1_1, Pins::E22_RXEN_1, Pins::E22_TXEN_1, Pins::E22_RESET_1);
    pinMode(Pins::LED_BLUE, OUTPUT);
    pinMode(Pins::LED_GREEN, OUTPUT);
    pinMode(Pins::LED_ORANGE, OUTPUT);
    pinMode(Pins::LED_RED, OUTPUT);
    digitalWrite(Pins::LED_RED, HIGH);

    SPI0.begin(Pins::SPI_SCK_0, Pins::SPI_MISO_0, Pins::SPI_MOSI_0);
    SPI1.begin(Pins::SPI_SCK_1, Pins::SPI_MISO_1, Pins::SPI_MOSI_1);
    
    if(!init_radio(Radio0, RADIO0_FREQ)) Serial.println(json_init_failure);
    if(!init_radio(Radio1, RADIO1_FREQ)) Serial.println(json_init_failure);
    Serial.println(json_init_success);
    digitalWrite(Pins::LED_RED, LOW);
    digitalWrite(Pins::LED_GREEN, HIGH);


    RadioConfig radio0_cfg{
        .radio=&Radio0,
        .cmd_queue=&radio0_cmds,
        .frequency=RADIO0_FREQ,
        .serial=0,
        .indicator_led=Pins::LED_ORANGE,
    };

    RadioConfig radio1_cfg{
        .radio=&Radio1,
        .cmd_queue=&radio1_cmds,
        .frequency=RADIO1_FREQ,
        .serial=1,
        .indicator_led=Pins::LED_BLUE,
    };

    xTaskCreatePinnedToCore(Radio_Rx_Thread, "Radio0_thread", 8192, &radio0_cfg, 0, nullptr, 1);
    xTaskCreatePinnedToCore(Radio_Rx_Thread, "Radio1_thread", 8192, &radio1_cfg, 0, nullptr, 1);
    xTaskCreatePinnedToCore(Management_Thread, "Management_thread", 8192, nullptr, 0, nullptr, 1);
    while(true) {
        delay(10000);
    }
}

void loop() {
    delay(10000);
}