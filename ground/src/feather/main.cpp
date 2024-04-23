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
// #define RFM95_EN
#define RFM95_INT 3
// #define LED 13 // Blinks on receipt

/* Pins for Teensy 31*/
// Ensure to change depending on wiring
// #define RFM95_CS 10
// #define RFM95_RST 15
// #define RFM95_EN 14
// #define RFM95_INT 16
// #define LED 13 // Blinks on receipt

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 104.0

#define DEFAULT_CMD 0
#define MAX_CMD_LEN 10

typedef uint32_t systime_t;
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
// For reading from
char incomingCmd[MAX_CMD_LEN];
char curByte;
short charIdx = 0;
short readySend = 0;
int command_ID = 0;
short cmd_number = 0;

template <typename T>
float convert_range(T val, float range) {
    size_t numeric_range = (int64_t)std::numeric_limits<T>::max() - (int64_t)std::numeric_limits<T>::min() + 1;
    return val * range / (float)numeric_range;
}

struct TelemetryPacket {
    int32_t lat;
    int32_t lon;
    uint16_t alt;
    uint16_t baro_alt;
    uint16_t highg_ax; //14 bit signed ax [-16,16) 2 bit tilt angle
    uint16_t highg_ay;  //1bit sign 13 bit unsigned [0,16) 2 bit tilt angle
    uint16_t highg_az;  //1bit sign 13 bit unsigned [0,16) 2 bit tilt angle
    uint8_t batt_volt;
    uint8_t fsm_satcount;
};

struct FullTelemetryData {
    systime_t timestamp;  //[0, 2^32]
    uint16_t barometer_altitude; // [0, 4096]
    float latitude; // [-90, 90]
    float longitude; // [-180, 180]
    float highG_ax; // [-16, 16]
    float highG_ay; // [-16, 16]
    float highG_az; // [-16, 16]
    float battery_voltage; // [0, 5]
    uint8_t FSM_State; // [0, 255]
};
enum class CommandType { SET_FREQ, SET_CALLSIGN, ABORT, TEST_FLAP, EMPTY };
// Commands transmitted from ground station to rocket
struct telemetry_command {
    CommandType command;
    int id;
    union {
        char callsign[8];
        float freq;
        bool do_abort;
    };
    std::array<char, 6> verify = {{'A', 'Y', 'B', 'E', 'R', 'K'}};
};

struct TelemetryCommandQueueElement {
    telemetry_command command;
    int retry_count;
};

std::queue<TelemetryCommandQueueElement> cmd_queue;
std::queue<FullTelemetryData> print_queue;

constexpr const char* json_command_success = R"({"type": "command_success"})";
constexpr const char* json_command_parse_error = R"({"type": "command_error", "error": "serial parse error"})";
constexpr const char* json_buffer_full_error = R"({"type": "command_error", "error": "command buffer not empty"})";

constexpr const char* json_init_failure = R"({"type": "init_error", "error": "failed to initilize LORA"})";
constexpr const char* json_init_success = R"({"type": "init_success"})";
constexpr const char* json_set_frequency_failure = R"({"type": "freq_error", "error": "set_frequency failed"})";
constexpr const char* json_receive_failure = R"({"type": "receive_error", "error": "recv failed"})";
constexpr const char* json_send_failure = R"({"type": "send_error", "error": "command_retries_exceded"})";
constexpr int max_command_retries = 5;

float current_freq = RF95_FREQ;

void printFloat(float f, int precision = 5) {
    if (isinf(f) || isnan(f)) {
        Serial.print(-1);
    } else {
        Serial.print(f, precision);
    }
}

void EnqueuePacket(const TelemetryPacket& packet, float frequency) {

    int64_t start_printing = millis();

    FullTelemetryData data{};
    data.timestamp = start_printing;
    data.barometer_altitude = packet.alt;
    data.latitude = convert_range(packet.lat, 180);
    data.longitude = convert_range(packet.lon, 360);
    data.highG_ax = convert_range(packet.highg_ax, 32);
    data.highG_ay = convert_range(packet.highg_ay, 32);
    data.highG_az = convert_range(packet.highg_az, 32);
    data.battery_voltage = convert_range(packet.batt_volt, 5);
    data.FSM_State = packet.fsm_satcount;
}

void printJSONField(const char* name, float val, bool comma = true) {
    Serial.print('\"');
    Serial.print(name);
    Serial.print("\":");
    printFloat(val);
    if (comma) Serial.print(',');
}

void printJSONField(const char* name, int val, bool comma = true) {
    Serial.print('\"');
    Serial.print(name);
    Serial.print("\":");
    Serial.print(val);
    if (comma) Serial.print(',');
}

void printJSONField(const char* name, const char* val, bool comma = true) {
    Serial.print('\"');
    Serial.print(name);
    Serial.print("\":\"");
    Serial.print(val);
    Serial.print('"');
    if (comma) Serial.print(',');
}

void printPacketJson(FullTelemetryData const& packet) {
    Serial.print(R"({"type": "data", "value": {)");
    printJSONField("timestamp", packet.timestamp);
    printJSONField("barometer_altitude", packet.barometer_altitude);
    printJSONField("latitude", packet.latitude);
    printJSONField("longitude", packet.longitude);
    printJSONField("highG_ax", packet.highG_ax);
    printJSONField("highG_ay", packet.highG_ay);
    printJSONField("highG_az", packet.highG_az);
    printJSONField("battery_voltage", packet.battery_voltage);
    printJSONField("FSM_State", packet.FSM_State, false);
    Serial.println("}}");
}

void PrintDequeue() {
    if (print_queue.empty()) return;

    auto packet = print_queue.front();
    print_queue.pop();
    printPacketJson(packet);
}

void SerialError() { Serial.println(json_command_parse_error); }

void set_freq_local_bug_fix(float freq) {
    telemetry_command t;
    t.command = CommandType::EMPTY;
    rf95.send((uint8_t*)&t, sizeof(t));
    rf95.waitPacketSent();
    rf95.setFrequency(freq);
    current_freq = freq;
}

void SerialInput(const char* key, const char* value) {
    /* If queue is not empty, do not accept new command*/
    if (!cmd_queue.empty()) {
        Serial.println(json_buffer_full_error);
        return;
    }

    telemetry_command command{};
    if (strcmp(key, "ABORT") == 0) {
        command.command = CommandType::ABORT;
        command.do_abort = true;
    } else if (strcmp(key, "FREQ") == 0) {
        command.command = CommandType::SET_FREQ;
        float v = atof(value);
        command.freq = min(max(v, 390), 445);
    } else if (strcmp(key, "CALLSIGN") == 0) {
        command.command = CommandType::SET_CALLSIGN;
        memset(command.callsign, ' ', sizeof(command.callsign));
        memcpy(command.callsign, value, min(strlen(value), sizeof(command.callsign)));
    } else if (strcmp(key, "FLOC") == 0) {
        float v = atof(value);
        v = min(max(v, 390), 445);
        set_freq_local_bug_fix(v);
        Serial.println(json_command_success);
        Serial.print(R"({"type": "freq_success", "frequency":)");
        Serial.print(v);
        Serial.println("}");
        return;
    } else if (strcmp(key, "FLAP") == 0) {
        command.command = CommandType::TEST_FLAP;
    } else {
        SerialError();
        return;
    }
    Serial.println(json_command_success);
    command_ID++;
    command.id = command_ID;
    cmd_queue.push({command, 0});
}

void process_command_queue() {
    if (cmd_queue.empty()) return;

    TelemetryCommandQueueElement cmd = cmd_queue.front();
    rf95.send((uint8_t*)&cmd.command, sizeof(cmd.command));
    rf95.waitPacketSent();
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
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println(json_set_frequency_failure);
        while (1)
            ;
    }

    Serial.print(R"({"type": "freq_success", "frequency":)");
    Serial.print(RF95_FREQ);
    Serial.println("}");

    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf =
    // 128chips/symbol, CRC on

    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST
    // transmitter pin, then you can set transmitter powers from 5 to 23 dBm:
    rf95.setTxPower(23, false);
}

void loop() {
    PrintDequeue();
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim(); // Remove leading/trailing whitespace

        // Check if input is a frequency value
        float frequency = input.toFloat();
        if (frequency >= 390 && frequency <= 445) { // Check if within valid range
            // Set the LoRa frequency
            rf95.setFrequency(frequency);
            current_freq = frequency;

            // Send success message over serial
            Serial.print(R"({"type": "freq_success", "frequency":)");
            Serial.print(frequency);
            Serial.println("}");
        } else {
            // Send error message over serial for invalid frequency
            Serial.println(R"({"type": "freq_error", "error": "invalid frequency"})");
        }
    }
    // static float f = 0;
    // static float f2 = 0;
    // f+=0.1;
    // f2 += 0.01;
    // if(f > 3.14) f -= 6.28;
    // delay(30);
    // FullTelemetryData d{};
    // d.barometer_pressure = 1000;
    // d.barometer_temp = 20;
    // d.bno_pitch = cos(f2);
    // d.bno_roll = sin(f2);
    // d.bno_yaw = 0;
    // d.flap_extension = f / 10;
    // d.freq = 434;
    // d.FSM_State = 3;
    // d.gnc_state_ax = f * 100;
    // d.gnc_state_vx = f * 10;
    // d.gnc_state_x = f * 1000;
    // d.gnc_state_apo = 100;
    // d.gps_alt = 1000+100*f;
    // d.gps_lat = 40;
    // d.gps_long = 80 + f;
    // d.gyro_x = sin(f2);
    // d.gyro_y = f+30;
    // d.gyro_z = f+40;
    // d.highG_ax = 10+f;
    // d.highG_ay = f/10;
    // d.highG_az = f/10;
    // d.mag_x = f;
    // d.mag_y = f+1;
    // d.mag_z = f+2;
    // d.voltage_battery = f + 4;
    // printPacketJson(d);

    if (rf95.available()) {
        // Should be a message for us now
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        // telemetry_data data{};
        TelemetryPacket packet;
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) {
            // Serial.println(len);
            // Serial.println("Received packet");
            // Serial.println(packet.datapoints[0].barometer_pressure);
            
            digitalWrite(LED_BUILTIN, HIGH);
            delay(50);
            digitalWrite(LED_BUILTIN, LOW);
            memcpy(&packet, buf, sizeof(packet));
            EnqueuePacket(packet, current_freq);
            // Serial.print("Queue size: ");
            // Serial.println((print_queue.size()));

            if (!cmd_queue.empty()) {
                auto& cmd = cmd_queue.front();
                // if (cmd.command.id == packet.response_ID) {
                //     if (cmd.command.command == CommandType::SET_FREQ) {
                //         set_freq_local_bug_fix(cmd.command.freq);
                //         Serial.print(R"({"type": "freq_success", "frequency":)");
                //         Serial.print(cmd.command.freq);
                //         Serial.println("}");
                //     }
                //     cmd_queue.pop();
                // } else {
                    cmd.retry_count++;
                    if (cmd.retry_count >= max_command_retries) {
                        cmd_queue.pop();
                        Serial.println(json_send_failure);
                    }
                // }
            }

            process_command_queue();

        } else {
            Serial.println(json_receive_failure);
        }
    }
    serial_parser.read();
}