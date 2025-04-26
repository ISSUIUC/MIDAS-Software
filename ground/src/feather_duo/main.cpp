#include<Arduino.h>
#include<SPI.h>
#include"FreeRTOS.h"
#include"E22.h"
#include"duo_pins.h"
#include"Packet.h"
#include"Output.h"
#include"Command.h"
#include"Queue.h"

constexpr uint32_t BOOSTER_FREQ = 426150000;
constexpr uint32_t SUSTAINER_FREQ = 421150000;
Queue<TelemetryCommand> booster_cmds;
Queue<TelemetryCommand> sustainer_cmds;

enum class Stage {
    Booster,
    Sustainer
};

struct RadioConfig
{
    SX1268 * radio;
    Queue<TelemetryCommand>* cmd_queue;
    uint32_t frequnecy;
    Stage stage;
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
    TelemetryCommand to_send;
    to_send.command = CommandType::EMPTY;

    while(true) {
        TelemetryPacket packet{};
        SX1268Error res = cfg->radio->recv((uint8_t*)&packet, sizeof(packet), -1);
        if(res == SX1268Error::NoError) {
            led_state = !led_state;
            digitalWrite(cfg->indicator_led, led_state);
            FullTelemetryData data = DecodePacket(packet, cfg->frequnecy / 1e6);
            data.rssi = cfg->radio->get_last_snr();
            printPacketJson(data);

            if(data.kf_reset != reset_state) {
                data.kf_reset = reset_state;
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
    Stage stage;
    switch(key[0]) {
        case '0':
            stage = Stage::Booster;
            break;
        case '1':
            stage = Stage::Sustainer;
            break;
        default:
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
    } else if (cmd_name == "CAMT") {
        command.command = CommandType::CAM_TOGGLE;
    } else {
        Serial.println(json_command_bad);
        return;
    }

    if(stage == Stage::Booster) {
        booster_cmds.send(command);
    }
    if(stage == Stage::Sustainer) {
        sustainer_cmds.send(command);
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
    
    if(!init_radio(Radio0, BOOSTER_FREQ)) Serial.println(json_init_failure);
    if(!init_radio(Radio1, SUSTAINER_FREQ)) Serial.println(json_init_failure);
    Serial.println(json_init_success);
    digitalWrite(Pins::LED_RED, LOW);
    digitalWrite(Pins::LED_GREEN, HIGH);


    RadioConfig booster_cfg{
        .radio=&Radio0,
        .cmd_queue=&booster_cmds,
        .frequnecy=BOOSTER_FREQ,
        .stage=Stage::Booster,
        .indicator_led=Pins::LED_ORANGE,
    };

    RadioConfig sustainer_cfg{
        .radio=&Radio1,
        .cmd_queue=&sustainer_cmds,
        .frequnecy=SUSTAINER_FREQ,
        .stage=Stage::Sustainer,
        .indicator_led=Pins::LED_BLUE,
    };

    xTaskCreatePinnedToCore(Radio_Rx_Thread, "Radio0_thread", 8192, &booster_cfg, 0, nullptr, 1);
    xTaskCreatePinnedToCore(Radio_Rx_Thread, "Radio1_thread", 8192, &sustainer_cfg, 0, nullptr, 1);
    xTaskCreatePinnedToCore(Management_Thread, "Managmenet_thread", 8192, nullptr, 0, nullptr, 1);
    while(true) {
        delay(10000);
    }
}

void loop() {
    delay(10000);
}