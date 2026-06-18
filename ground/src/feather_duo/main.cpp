#include<Arduino.h>
#include<SPI.h>
#include"FreeRTOS.h"
#include"E22.h"
#include"duo_pins.h"
#include"Packet.h"
#include"Output.h"
#include"Command.h"
#include"Queue.h"
#include "midas_shell_commands.h"
#include "esp_eeprom.h"
#include "duo.h"

Queue<TelemetryCommand> radio0_cmds;
Queue<TelemetryCommand> radio1_cmds;



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
    TelemetryCommand to_send;
    to_send.command = CommandType::EMPTY;

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
                to_send.setSerial(cfg->serial);
                (void)cfg->radio->send((uint8_t*)&to_send, sizeof(to_send));
                Serial.println(json_command_sent);
            }
        }

        if(cfg->desired_frequency != cfg->frequency){
            cfg->frequency = cfg->desired_frequency;
            cfg->radio->set_frequency(cfg->frequency);
        }

        if(cfg->desired_serial != cfg->serial){
            cfg->serial = cfg->desired_serial;
        }
    }
}


void Shell_Thread(void *arg)
{
    DuoSystems* systems = (DuoSystems*)arg;

    char line_buf[MShell::max_line_len];
    char tmp_buf[MShell::max_line_len];
    uint8_t d_read = 0;

    while(true) {
        // if(shell->settings.echo) {
        //     Serial.print("> ");
        //     Serial.flush();
        // }
        int num_bytes_avail = Serial.available();

        if(num_bytes_avail > 0) {
            Serial.read(tmp_buf, num_bytes_avail);
            for(int i = 0; i < num_bytes_avail; i++) {

                if(tmp_buf[i] == '\r') {
                    continue;
                }

                if(tmp_buf[i] == '\b' || tmp_buf[i] == 0x7F) {
                    if(d_read > 0) {
                        d_read--;
                        if(systems->shell->settings.echo) {
                            Serial.print("\b \b"); // erase character on terminal
                        }
                    }
                    continue;
                }

                if(systems->shell->settings.echo) {
                    Serial.print(tmp_buf[i]);
                }

                if(tmp_buf[i] == '\n') {
                    // Process the line instead of adding it!
                    line_buf[d_read] = '\0'; // Null terminate command
                    d_read = 0;

                    MCommandExecutionResult c_res = systems->shell->execute_line(line_buf, systems);
                    Serial.print("<done> ");
                    if (c_res == MCommandExecutionResult::OK) { Serial.println(json_command_success); }
                    else { Serial.println(json_command_bad); }

                    if(systems->shell->settings.echo) {
                        Serial.print("> ");
                        Serial.flush();
                    }

                    continue;
                }

                if(d_read >= MShell::max_line_len - 1) {
                    continue;
                }

                // Otherwise add to the buf
                line_buf[d_read++] = tmp_buf[i];
            }
            Serial.flush();
        }

        delay(10);
    }
}


void setup() {
    Serial.begin(460800);

    DuoSystems systems;
    
    SPIClass SPI0(HSPI);
    SPIClass SPI1(FSPI);
    SX1268 Radio0(SPI0, Pins::E22_CS_0, Pins::E22_BUSY_0, Pins::E22_DIO1_0, Pins::E22_RXEN_0, Pins::E22_TXEN_0, Pins::E22_RESET_0);
    SX1268 Radio1(SPI1, Pins::E22_CS_1, Pins::E22_BUSY_1, Pins::E22_DIO1_1, Pins::E22_RXEN_1, Pins::E22_TXEN_1, Pins::E22_RESET_1);
    pinMode(Pins::LED_BLUE, OUTPUT);
    pinMode(Pins::LED_GREEN, OUTPUT);
    pinMode(Pins::LED_ORANGE, OUTPUT);
    pinMode(Pins::LED_RED, OUTPUT);
    digitalWrite(Pins::LED_RED, LOW);

    SPI0.begin(Pins::SPI_SCK_0, Pins::SPI_MISO_0, Pins::SPI_MOSI_0);
    SPI1.begin(Pins::SPI_SCK_1, Pins::SPI_MISO_1, Pins::SPI_MOSI_1);

    if(!systems.eeprom.init()) {
        digitalWrite(Pins::LED_RED, HIGH);
        while(true);
    }

    m_shell_setup(); // Set up the MIDAS shell
    systems.shell = &m_shell_inst;
    m_shell_init_commands(systems.shell);
    
    if(!init_radio(Radio0, systems.eeprom.data.frequency[0])){
        Serial.println(json_init_failure);
        digitalWrite(Pins::LED_RED, HIGH);
        while(true);
    } 
    if(!init_radio(Radio1, systems.eeprom.data.frequency[1])) {
        Serial.println(json_init_failure);
        digitalWrite(Pins::LED_RED, HIGH);
        while(true);
    }
    Serial.println(json_init_success);
    digitalWrite(Pins::LED_GREEN, HIGH);

    RadioConfig radio0_cfg{
        .radio=&Radio0,
        .cmd_queue=&radio0_cmds,
        .frequency=systems.eeprom.data.frequency[0],
        .desired_frequency=systems.eeprom.data.frequency[0],
        .serial=systems.eeprom.data.serial[0],
        .desired_serial=systems.eeprom.data.serial[0],
        .indicator_led=Pins::LED_ORANGE,
    };

    RadioConfig radio1_cfg{
        .radio=&Radio1,
        .cmd_queue=&radio1_cmds,
        .frequency=systems.eeprom.data.frequency[1],
        .desired_frequency=systems.eeprom.data.frequency[1],
        .serial=systems.eeprom.data.serial[1],
        .desired_serial=systems.eeprom.data.serial[1],
        .indicator_led=Pins::LED_BLUE,
    };

    systems.cfg[0] = radio0_cfg;
    systems.cfg[1] = radio1_cfg;

    xTaskCreatePinnedToCore(Radio_Rx_Thread, "Radio0_thread", 8192, &systems.cfg[0], 0, nullptr, 1);
    xTaskCreatePinnedToCore(Radio_Rx_Thread, "Radio1_thread", 8192, &systems.cfg[1], 0, nullptr, 1);
    xTaskCreatePinnedToCore(Shell_Thread, "Shell_thread", 8192, &systems, 0, nullptr, 1);
    while(true) {
        delay(10000);
    }
}

void loop() {
    delay(10000);
}