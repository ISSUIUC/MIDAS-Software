/**
 * This file contains the code than runs on our ground station
 * hardware (LoRa Feather module). It includes the receive code,
 * command functionality, and interfaces with the ground station GUI
 * through serial.
 *
 * Spaceshot Telemetry Team 2023-24
 * Nicholas Phillips
 * Gautam Dayal
 * Patrick Marschoun
 * Peter Giannetos
 * Aaditya Voruganti
 */

#include <RH_RF95.h>
#include <SPI.h>

#include <array>
#include <limits>
#include <numeric>
#include <queue>

#include "SerialParser.h"

/* Pins for feather*/
// // Ensure to change depending on wiring
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
float rf95_freq_MHZ = 425.15;

#define DEFAULT_CMD 0
#define MAX_CMD_LEN 10

typedef uint32_t systime_t;
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
// For reading from
bool last_ack_bit = false;

template <typename T>
float convert_range(T val, float range) {
    size_t numeric_range = (int64_t)std::numeric_limits<T>::max() - (int64_t)std::numeric_limits<T>::min() + 1;
    return val * range / (float)numeric_range;
}


/**
 * @struct TelemetryPacket
 * 
 * @brief format of the telemetry packet
*/
struct TelemetryPacket {
    int32_t lat;
    int32_t lon;
    uint16_t alt; //15 bit meters, 1 bit last command confirm
    uint16_t baro_alt;
    uint16_t highg_ax; //14 bit signed ax [-16,16) 2 bit tilt angle
    uint16_t highg_ay; //14 bit signed ax [-16,16) 2 bit tilt angle
    uint16_t highg_az; //14 bit signed ax [-16,16) 2 bit tilt angle
    uint8_t batt_volt;
    
    
    // If callsign bit (highest bit of fsm_callsign_satcount) is set, the callsign is KD9ZMJ
    //
    // If callsign bit (highest bit of fsm_callsign_satcount) is not set, the callsign is KD9ZPM
    
    uint8_t fsm_callsign_satcount; //4 bit fsm state, 1 bit is_sustainer_callsign, 3 bits sat count
};

enum class CommandType: uint8_t { RESET_KF, SWITCH_TO_SAFE, SWITCH_TO_PYRO_TEST, SWITCH_TO_IDLE, FIRE_PYRO_A, FIRE_PYRO_B, FIRE_PYRO_C, FIRE_PYRO_D };
// Commands transmitted from ground station to rocket
struct TelemetryCommand {
    CommandType command;
    union {
        float new_freq;
    };
    std::array<char, 3> verify = {{'B', 'R', 'K'}};
};
static_assert(sizeof(TelemetryCommand) == 12);

struct TelemetryCommandQueueElement {
    TelemetryCommand command;
    int retry_count;
};

std::queue<TelemetryCommandQueueElement> cmd_queue;

constexpr const char* json_command_success = R"({"type": "command_success"})";
constexpr const char* json_command_parse_error = R"({"type": "command_error", "error": "serial parse error"})";
constexpr const char* json_buffer_full_error = R"({"type": "command_error", "error": "command buffer not empty"})";

constexpr const char* json_init_failure = R"({"type": "init_error", "error": "failed to initilize LORA"})";
constexpr const char* json_init_success = R"({"type": "init_success"})";
constexpr const char* json_set_frequency_failure = R"({"type": "freq_error", "error": "set_frequency failed"})";
constexpr const char* json_receive_failure = R"({"type": "receive_error", "error": "recv failed"})";
constexpr const char* json_send_failure = R"({"type": "send_error", "error": "command_retries_exceded"})";
constexpr int max_command_retries = 5;

float prev_freq = rf95_freq_MHZ;

void printFloat(float f, int precision = 5) {
    if (isinf(f) || isnan(f)) {
        Serial.print(-1);
    } else {
        Serial.print(f, precision);
    }
}

void SerialError() { Serial.println(json_command_parse_error); }

void SerialInput(const char* key, const char* value) {
    /* If queue is not empty, do not accept new command*/
    if (!cmd_queue.empty()) {
        Serial.println(json_buffer_full_error);
        return;
    }

    TelemetryCommand command{};
    // if (strcmp(key, "RESET_KF") == 0) {
    //     command.command = CommandType::RESET_KF;
    // } else if (strcmp(key, "SET_FREQ") == 0) {
    //     command.command = CommandType::SET_FREQ;
    //     command.new_freq = atof(value);
    // } else if (strcmp(key, "EN_PYRO") == 0) {
    //     command.command = CommandType::EN_PYRO;
    // } else if (strcmp(key, "DIS_PYRO") == 0) {
    //     command.command = CommandType::DIS_PYRO;
    // } else {
    //     SerialError();
    //     return;
    // }

    if (strcmp(key, "RESET_KF") == 0) {
        command.command = CommandType::RESET_KF;
    } else if (strcmp(key, "SAFE") == 0) {
        command.command = CommandType::SWITCH_TO_SAFE;
    } else if (strcmp(key, "IDLE") == 0) {
        command.command = CommandType::SWITCH_TO_IDLE;
    } else if (strcmp(key, "PT") == 0) {
        command.command = CommandType::SWITCH_TO_PYRO_TEST;
    } else if (strcmp(key, "PA") == 0) {
        command.command = CommandType::FIRE_PYRO_A;
    } else if (strcmp(key, "PB") == 0) {
        command.command = CommandType::FIRE_PYRO_B;
    } else if (strcmp(key, "PC") == 0) {
        command.command = CommandType::FIRE_PYRO_C;
    } else if (strcmp(key, "PD") == 0) {
        command.command = CommandType::FIRE_PYRO_D;
    } else {
        // SerialError();
        Serial.println("bad command");
        return;
        // return;
    }

    
    Serial.println(json_command_success);
    // Send the command until acknowledge or 5 attempts
    cmd_queue.push({command, 5});
}

void handle_acknowledge() {
    if (!cmd_queue.empty()) {
        cmd_queue.pop();
    }
}

void process_command_queue() {
    if (cmd_queue.empty()) return;

    TelemetryCommandQueueElement& cmd = cmd_queue.front();
    cmd.retry_count --;
    rf95.send((uint8_t*)&cmd.command, sizeof(cmd.command));
    rf95.waitPacketSent();
    // if (cmd.command.command == CommandType::SET_FREQ) {
    //     prev_freq = rf95_freq_MHZ;
    //     rf95_freq_MHZ = cmd.command.new_freq;
    //     rf95.setFrequency(rf95_freq_MHZ);
    //     Serial.println(rf95_freq_MHZ);
    // }
    // if(cmd.retry_count <= 0) {
    //     if (cmd.command.command == CommandType::SET_FREQ) {
    //         rf95_freq_MHZ = prev_freq;
    //         rf95.setFrequency(rf95_freq_MHZ);
    //     }
    //     Serial.printf(json_send_failure);
    //     cmd_queue.pop();
    // }
}

SerialParser serial_parser(SerialInput, SerialError);

void setup() {
    while (!Serial)
        ;
    Serial.begin(9600);

    if (!rf95.init()) {
        Serial.println(json_init_failure);
        while (1)
            ;
    }

    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println(json_init_success);

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(rf95_freq_MHZ)) {
        Serial.println(json_set_frequency_failure);
        while (1)
            ;
    }

    Serial.print(R"({"type": "freq_success", "frequency":)");
    Serial.print(rf95_freq_MHZ);
    Serial.println("}");

    rf95.setSignalBandwidth(125000);
    rf95.setCodingRate4(8);
    rf95.setSpreadingFactor(10);
    rf95.setPayloadCRC(true);
    rf95.setTxPower(23, false);
}

void loop() {
    if (rf95.available()) {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        TelemetryPacket packet;
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) {
            if(len != sizeof(packet)) {
                Serial.println("Received someone else's transmission");
                return;
            }

            Serial.println("Received packet");
            memcpy(&packet, buf, sizeof(packet));

            if(bool(packet.alt & 0x1) != last_ack_bit) {
                last_ack_bit = !last_ack_bit;
                Serial.println("Command Acknowledged");
                handle_acknowledge();
            }
            process_command_queue();
        } else {
            Serial.println(json_receive_failure);
        }
    }
    serial_parser.read();
}