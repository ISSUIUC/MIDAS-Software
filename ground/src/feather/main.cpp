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
 * Micheal Karpov
 */

#include <RH_RF95.h>
#include <SPI.h>

#include <array>
#include <limits>
#include <numeric>
#include <queue>
#include <algorithm>

#include "SerialParser.h"

/* Pins for feather*/
// // Ensure to change depending on wiring
#define RFM95_CS 8
#define RFM95_RST 4
// #define RFM95_EN
#define RFM95_INT 3
#define VoltagePin 14
// #define LED 13 // Blinks on receipt

float RF95_FREQ = 425.15;
float SUSTAINER_FREQ = 425.15;

float BOOSTER_FREQ = 425.15;
float GROUND_FREQ = 420;
float rf95_freq_MHZ = 425.15;

float current_freq = 0;

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

constexpr const char* json_command_success = R"({"type": "command_success"})";
constexpr const char* json_command_bad = R"({"type": "bad_command"})";
constexpr const char* json_command_sent = R"({"type": "command_sent"})";
constexpr const char* json_command_ack = R"({"type": "command_acknowledge"})";
constexpr const char* json_command_parse_error = R"({"type": "command_error", "error": "serial parse error"})";
constexpr const char* json_buffer_full_error = R"({"type": "command_error", "error": "command buffer not empty"})";

constexpr const char* json_init_failure = R"({"type": "init_error", "error": "failed to initilize LORA"})";
constexpr const char* json_init_success = R"({"type": "init_success"})";
constexpr const char* json_set_frequency_failure = R"({"type": "freq_error", "error": "set_frequency failed"})";
constexpr const char* json_receive_failure = R"({"type": "receive_error", "error": "recv failed"})";
constexpr const char* json_send_failure = R"({"type": "send_error", "error": "command_retries_exceded"})";
constexpr int max_command_retries = 20;

bool last_ack_bit = false;
bool initial_ack_flag = true;


template <typename T>
float convert_range(T val, float range) {
    size_t numeric_range = (int64_t)std::numeric_limits<T>::max() - (int64_t)std::numeric_limits<T>::min() + 1;
    return val * range / (float)numeric_range;
}

struct TelemetryPacket {
    int32_t lat;
    int32_t lon;

    uint16_t alt; //15 bit meters, 1 bit last command confirm
    uint16_t baro_alt;
    uint16_t highg_ax; //14 bit signed ax [-16,16) 2 bit tilt angle
    uint16_t highg_ay;  //14 bit signed ax [-16,16) 2 bit tilt angle
    uint16_t highg_az;  //14 bit signed ax [-16,16) 2 bit tilt angle

    uint8_t batt_volt;
    
    // If callsign bit (highest bit of fsm_callsign_satcount) is set, the callsign is KD9ZMJ
    //
    // If callsign bit (highest bit of fsm_callsign_satcount) is not set, the callsign is KD9ZPM
    
    uint8_t fsm_callsign_satcount; //4 bit fsm state, 1 bit is_sustainer_callsign, 3 bits sat count
    uint16_t kf_vx; // 16 bit meters/second
    uint32_t pyro; // 7 bit continuity 4 bit tilt
    float RSSI;
};

// struct TelemetryPacket {
//     int32_t lat;
//     int32_t lon;
//     uint16_t alt; //15 bit meters, 1 bit last command confirm
//     uint16_t baro_alt;
//     uint16_t highg_ax; //14 bit signed ax [-16,16) 2 bit tilt angle
//     uint16_t highg_ay; //14 bit signed ax [-16,16) 2 bit tilt angle
//     uint16_t highg_az; //14 bit signed ax [-16,16) 2 bit tilt angle
//     uint8_t batt_volt;
//     // If callsign bit (highest bit of fsm_callsign_satcount) is set, the callsign is KD9ZMJ
//     //
//     // If callsign bit (highest bit of fsm_callsign_satcount) is not set, the callsign is KD9ZPM
//     uint8_t fsm_callsign_satcount; //4 bit fsm state, 1 bit is_sustainer_callsign, 3 bits sat count
// };

struct FullTelemetryData {
    systime_t timestamp;  //[0, 2^32]
    uint16_t altitude; // [0, 4096]
    float latitude; // [-90, 90]
    float longitude; // [-180, 180]
    float barometer_altitude; // [0, 4096]
    float highG_ax; // [-16, 16]
    float highG_ay; // [-16, 16]
    float highG_az; // [-16, 16]
    float battery_voltage; // [0, 5]
    uint8_t FSM_State; // [0, 255]
    float tilt_angle; // [-90, 90]
    float freq;
    float rssi;
    float sat_count;
    float pyros[4];
    bool is_sustainer;
    float kf_vx;
    bool kf_reset;
};



enum class CommandType: uint8_t { RESET_KF, SWITCH_TO_SAFE, SWITCH_TO_PYRO_TEST, SWITCH_TO_IDLE, FIRE_PYRO_A, FIRE_PYRO_B, FIRE_PYRO_C, FIRE_PYRO_D, EMPTY };
// Commands transmitted from ground station to rocket
struct TelemetryCommand {
    CommandType command;
    union {
        char callsign[8];
        float freq;
        bool do_abort;
    };
    std::array<char, 3> verify = {{'B', 'R', 'K'}};
};

struct TelemetryCommandQueueElement {
    TelemetryCommand command;
    int retry_count;
};

std::queue<TelemetryCommandQueueElement> cmd_queue;
std::queue<FullTelemetryData> print_queue;

void printFloat(float f, int precision = 5) {
    if (isinf(f) || isnan(f)) {
        Serial.print(-1);
    } else {
        Serial.print(f, precision);
    }
}
int decodeLastTwoBits(uint16_t ax, uint16_t ay, uint16_t az) {
    int tilt_ax = ax & 0b11;
    int tilt_ay = ay & 0b11;
    int tilt_az = az & 0b11;
    int tilt = (tilt_ax << 0) | (tilt_ay << 2) | (tilt_az << 4);
    return tilt;
}

double ConvertGPS(int32_t coord) {
    double mins = fmod(static_cast<double>(std::abs(coord)), 10000000) / 100000.;
    double degs = floor(static_cast<double>(std::abs(coord)) / 10000000.);
    double complete = (degs + (mins / 60.));
    if (coord < 0) {
        complete *= -1.;
    }
    return complete;
}

void handle_acknowledge() {
    if (!cmd_queue.empty()) {
        cmd_queue.pop();
        Serial.println(json_command_ack);
    }
}

void EnqueuePacket(const TelemetryPacket& packet, float frequency) {

    int64_t start_printing = millis();

    FullTelemetryData data;
    data.altitude = static_cast<float>(packet.alt);
    data.latitude = ConvertGPS(packet.lat);
    data.longitude = ConvertGPS(packet.lon);
    data.barometer_altitude = convert_range<int16_t>(packet.baro_alt, 1 << 17);
    int tilt = decodeLastTwoBits(packet.highg_ax, packet.highg_ay, packet.highg_az);
    tilt |= (packet.pyro >> 28 & (0xF)) << 6;
    int16_t ax = packet.highg_ax & 0xfffc;
    int16_t ay = packet.highg_ay & 0xfffc;
    int16_t az = packet.highg_az & 0xfffc;
    data.highG_ax = convert_range<int16_t>(ax, 32);
    data.highG_ay = convert_range<int16_t>(ay, 32);
    data.highG_az = convert_range<int16_t>(az, 32);
    data.tilt_angle = tilt / 1023. * 180; // Returns tilt angle in range [0, 180]
    data.battery_voltage = convert_range(packet.batt_volt, 16);
    data.sat_count = packet.fsm_callsign_satcount >> 4 & 0b0111;
    data.is_sustainer = (packet.fsm_callsign_satcount >> 7);
    data.FSM_State = packet.fsm_callsign_satcount & 0b1111;
    data.pyros[0] = (packet.pyro >> 0) & (0x7F);
    data.pyros[1] = ((float) ((packet.pyro >> 7) & (0x7F)) / 127.) * 12.;
    data.pyros[2] = ((float) ((packet.pyro >> 14) & (0x7F)) / 127.) * 12.;
    data.pyros[3] = ((float) ((packet.pyro >> 21) & (0x7F)) / 127.) * 12.;
    data.kf_reset = packet.alt & 1;

    if(initial_ack_flag) {
        last_ack_bit = data.kf_reset;
        initial_ack_flag = false;
    }

    if(data.kf_reset != last_ack_bit) {
        handle_acknowledge();
        last_ack_bit = data.kf_reset;
    }
    // kinda hacky but it will work
    if (packet.fsm_callsign_satcount == static_cast<uint8_t>(-1)) {
        data.FSM_State = static_cast<uint8_t>(-1);
    }
    data.kf_vx = (float) packet.kf_vx / (float) ((1 << 16) - 1) * 4000.f - 2000.f;
    data.freq = RF95_FREQ;
    if(packet.RSSI == 0.0) {
        data.rssi = packet.RSSI;
    } else {
        data.rssi = rf95.lastRssi();
    }
    data.rssi = rf95.lastRssi();
    print_queue.emplace(data);

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

    bool is_heartbeat = packet.FSM_State == static_cast<uint8_t>(-1);

    Serial.print(R"({"type": ")");
    Serial.print(is_heartbeat ? "heartbeat" : "data");
    Serial.print(R"(", "value": {)");
    printJSONField("barometer_altitude", packet.barometer_altitude);
    printJSONField("latitude", packet.latitude);
    printJSONField("longitude", packet.longitude);
    printJSONField("altitude", packet.altitude);
    printJSONField("highG_ax", packet.highG_ax);
    printJSONField("highG_ay", packet.highG_ay);
    printJSONField("highG_az", packet.highG_az);
    printJSONField("battery_voltage", packet.battery_voltage);
    printJSONField("FSM_State", packet.FSM_State);
    printJSONField("tilt_angle", packet.tilt_angle);
    printJSONField("frequency", packet.freq);
    printJSONField("RSSI", packet.rssi);
    printJSONField("sat_count", packet.sat_count);
    printJSONField("kf_velocity", packet.kf_vx);
    printJSONField("is_sustainer", packet.is_sustainer);
    printJSONField("pyro_a", packet.pyros[0]);
    printJSONField("pyro_b", packet.pyros[1]);
    printJSONField("pyro_c", packet.pyros[2]);
    printJSONField("kf_reset", packet.kf_reset);
    printJSONField("pyro_d", packet.pyros[3], false);
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
    TelemetryCommand t;
    t.command = CommandType::EMPTY;
    rf95.send((uint8_t*)&t, 0);
    Serial.println(sizeof(t));
    rf95.waitPacketSent();
    rf95.setFrequency(freq);
    Serial.println(json_command_success);
    Serial.print(R"({"type": "freq_success", "frequency":)");
    Serial.print(freq);
    Serial.println("}");
}

void SerialInput(const char* key, const char* value) {
    if (!cmd_queue.empty()) {
        Serial.println(json_buffer_full_error);
        return;
    }

    TelemetryCommand command{};

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
        Serial.println(json_command_bad);
        return;
    }


    Serial.println(json_command_success);
    // Send the command until acknowledge or 5 attempts
    cmd_queue.push({command, 5});
}


void Stest(const String key) {
    if (!cmd_queue.empty()) {
        Serial.println(json_buffer_full_error);
        return;
    }

    TelemetryCommand command{};

    if (key == "RESET_KF") {
        command.command = CommandType::RESET_KF;
    } else if (key == "SAFE") {
        command.command = CommandType::SWITCH_TO_SAFE;
    } else if (key == "IDLE") {
        command.command = CommandType::SWITCH_TO_IDLE;
    } else if (key == "PT") {
        command.command = CommandType::SWITCH_TO_PYRO_TEST;
    } else if (key == "PA") {
        command.command = CommandType::FIRE_PYRO_A;
    } else if (key == "PB") {
        command.command = CommandType::FIRE_PYRO_B;
    } else if (key == "PC") {
        command.command = CommandType::FIRE_PYRO_C;
    } else if (key == "PD") {
        command.command = CommandType::FIRE_PYRO_D;
    } else {
        Serial.println(json_command_bad);
        return;
    }

    Serial.println(json_command_success);
    // Send the command until acknowledge or 5 attempts
    cmd_queue.push({command, 5});
}


void process_command_queue() {
    if (cmd_queue.empty()) return;
    TelemetryCommandQueueElement& cmd = cmd_queue.front();
    cmd.retry_count --;

    Serial.println(json_command_sent);
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
        while (1);
    }
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println(json_init_success);
    if (!rf95.setFrequency(rf95_freq_MHZ)) {
        Serial.println(json_set_frequency_failure);
        while (1);
    }
    
    current_freq = rf95_freq_MHZ;
    // #ifdef IS_DRONE
    // if (!rf95.setFrequency(SUSTAINER_FREQ)) {
    //     Serial.println(json_set_frequency_failure);
        
    //     while (1);

    // }
    // current_freq = SUSTAINER_FREQ;
    // #endif
    rf95.setCodingRate4(8);
    rf95.setSpreadingFactor(8);

    rf95.setPayloadCRC(true);

    rf95.setSignalBandwidth(250000);
    rf95.setPreambleLength(8);

    Serial.print(R"({"type": "freq_success", "frequency":)");
    Serial.print(current_freq);
    Serial.println("}");
    rf95.setTxPower(23, false);

    Serial.setTimeout(250);

}

void ChangeFrequency(float freq) {
    float current_time = millis();
    rf95.setFrequency(freq);
    Serial.println(json_command_success);
    Serial.print(R"({"type": "freq_success", "frequency":)");
    Serial.print(freq);
    Serial.println("}");
}

// void loop() {
//     TelemetryCommand tc;
//     tc.command = CommandType::SWITCH_TO_IDLE;
//     tc.freq = 1234.5678;

//     rf95.send((uint8_t*)&tc.command, sizeof(tc.command));

//     // rf95.send((uint8_t *) "greet thomas", strlen("hello thomas") + 1);
//     delay(1000);
//     rf95.handleInterrupt();
// }

int last_cmd_sent = 0;
String cur_input = "";

void loop() {
    PrintDequeue();

    // if(millis() - last_cmd_sent > 10000) {
    //     Serial.println("cmd enqueued");
    //     TelemetryCommand command{};
    //     command.command = CommandType::SWITCH_TO_SAFE;
    //     cmd_queue.push({command, 5});
    //     last_cmd_sent = millis();

    //     process_command_queue();
    // }

    // return;

    if (rf95.available()) {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        TelemetryPacket packet;
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(50);
            digitalWrite(LED_BUILTIN, LOW);

            // print out buf

            // for(unsigned i = 0; i < len; i++) {
            //     Serial.print(buf[i]);
            // }

            // Serial.println("   <eot>");

            // return;
            // Serial.println("Received packet");
            // Serial.println(len);
            memcpy(&packet, buf, sizeof(packet));
            EnqueuePacket(packet, current_freq);
            // if (!cmd_queue.empty()) {
            //     auto& cmd = cmd_queue.front();
            //         cmd.retry_count++;
            //         Serial.print("ATTEMPT #");
            //         Serial.println(cmd.retry_count);
            //         if (cmd.retry_count >= max_command_retries) {
            //             Serial.println("Popping command");
            //             cmd_queue.pop();
            //             Serial.println(json_send_failure);
            //         }
            // }

            process_command_queue();

        } else {
            Serial.println(json_receive_failure);
        }
    }

    // serial_parser.read();
    if (Serial.available()) {
        String input = Serial.readString();

        if(input.indexOf('\n') != -1) {
            cur_input += input;
            cur_input.replace("\r", ""); // Remove carriage returns

            Stest(input.substring(0, input.length() - 2));

            cur_input = "";
        } else {
            cur_input += input;
        }

        // if (input.startsWith("FREQ:")) {
        //     float freq = input.substring(5).toFloat(); // Extract frequency value
        //     set_freq_local_bug_fix(freq);
        //     RF95_FREQ = freq;
        //     current_freq = freq;
        // }

        // SerialInput(input.c_str(), "");

        // Avoid serialparser.
    }
}

// #ifdef IS_DRONE
// unsigned long prev_time = 0;
// unsigned long heartbeat_time = 0;

// uint8_t readBatteryVoltage() {
//     int batteryADC = analogRead(9);
//     float batteryVoltage = (batteryADC * 3.3 * 2) / 1024.0; //5.0Vmax
//     if (batteryVoltage > 5.0) {
//         batteryVoltage = 5.0;
//     }
//     if (batteryVoltage < 0.0) {
//         batteryVoltage = 0.0;
//     }

//     uint8_t battery = static_cast<uint8_t>((batteryVoltage/5.0)*255);
//     return battery;
// }

// void loop() {
    
//     PrintDequeue();
//     unsigned long current_time = millis();
//     if (current_time - prev_time > 2000) {
//         if(current_freq == SUSTAINER_FREQ) {
//             ChangeFrequency(BOOSTER_FREQ);
//             current_freq = BOOSTER_FREQ;
//             Serial.println("Sustainer timeout, Switching to booster freq");
//         } else {
//             ChangeFrequency(SUSTAINER_FREQ);
//             current_freq = SUSTAINER_FREQ;
//             Serial.println("Booster timeout, Switching to sustainer freq");
//         }
//         prev_time = millis();
//     }
//     if(millis() - heartbeat_time > 2000) {
//         Serial.println("Heartbeat");
//         TelemetryPacket packet;
//         rf95.setFrequency(GROUND_FREQ);
//         packet.batt_volt = readBatteryVoltage();
//         packet.fsm_satcount = -1;
//         rf95.send((uint8_t*)&packet, sizeof(packet)); 
//         rf95.waitPacketSent();
//         rf95.setFrequency(current_freq);
//         heartbeat_time = millis();
//     }
//     if (rf95.available()) {
//         TelemetryPacket packet;
//         uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
//         uint8_t len = sizeof(buf);

//         if (rf95.recv(buf, &len)) {
//             Serial.print("Recieved ");
//             Serial.print(len);
//             Serial.print(" bytes on ");
//             Serial.println(current_freq);
            

//             digitalWrite(LED_BUILTIN, HIGH);
//             delay(50);
//             digitalWrite(LED_BUILTIN, LOW);
//             memcpy(&packet, buf, sizeof(packet));
//             packet.RSSI = rf95.lastRssi();
//             EnqueuePacket(packet, current_freq);
//             set_freq_local_bug_fix(GROUND_FREQ);
//             rf95.send((uint8_t*)&packet, sizeof(packet)); 

//             if(current_freq == SUSTAINER_FREQ) {
//                 set_freq_local_bug_fix(BOOSTER_FREQ);
//                 current_freq = BOOSTER_FREQ;
//                 Serial.println("Switching to booster freq");
//             } else {
//                 set_freq_local_bug_fix(SUSTAINER_FREQ);
//                 current_freq = SUSTAINER_FREQ;
//                 Serial.println("Switching to sust69ainer freq");
//             }
//             prev_time = millis();
//             // Serial.print(current_time);
//         } else {
//             Serial.println(json_receive_failure);
//         }
//     }
//     serial_parser.read();
// }
// #endif
