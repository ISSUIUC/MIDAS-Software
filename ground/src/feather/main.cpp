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

#include "SerialParser.h"

/* Pins for feather*/
// // Ensure to change depending on wiring
#define RFM95_CS 8
#define RFM95_RST 4
// #define RFM95_EN
#define RFM95_INT 3
// #define LED 13 // Blinks on receipt

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 426.15
#ifdef IS_DRONE
#define SUSTAINER_FREQ 426.15
#define BOOSTER_FREQ 427
#define GROUND_FREQ 425
#endif

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
constexpr const char* json_command_parse_error = R"({"type": "command_error", "error": "serial parse error"})";
constexpr const char* json_buffer_full_error = R"({"type": "command_error", "error": "command buffer not empty"})";

constexpr const char* json_init_failure = R"({"type": "init_error", "error": "failed to initilize LORA"})";
constexpr const char* json_init_success = R"({"type": "init_success"})";
constexpr const char* json_set_frequency_failure = R"({"type": "freq_error", "error": "set_frequency failed"})";
constexpr const char* json_receive_failure = R"({"type": "receive_error", "error": "recv failed"})";
constexpr const char* json_send_failure = R"({"type": "send_error", "error": "command_retries_exceded"})";
constexpr int max_command_retries = 5;


template <typename T>
float convert_range(T val, float range) {
    size_t numeric_range = (int64_t)std::numeric_limits<T>::max() - (int64_t)std::numeric_limits<T>::min() + 1;
    return static_cast<float>(val) * range / (float)numeric_range;
}

struct TelemetryPacket {
    int32_t lat;
    int32_t lon;
    int16_t alt;
    int16_t baro_alt;
    uint16_t highg_ax; //14 bit signed ax [-16,16) 2 bit tilt angle
    uint16_t highg_ay;  //1bit sign 13 bit unsigned [0,16) 2 bit tilt angle
    uint16_t highg_az;  //1bit sign 13 bit unsigned [0,16) 2 bit tilt angle
    uint8_t batt_volt;
    uint8_t fsm_satcount;
    #ifdef IS_DRONE
    bool IS_SUSTAINER;
    #endif
};

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

void EnqueuePacket(const TelemetryPacket& packet, float frequency) {

    int64_t start_printing = millis();

    FullTelemetryData data;
    data.timestamp = start_printing;

    data.altitude = static_cast<float>(packet.alt);
    data.latitude = ConvertGPS(packet.lat);
    data.longitude = ConvertGPS(packet.lon);
    data.barometer_altitude = convert_range<int16_t>(packet.baro_alt, 1 << 17);
    int tilt = decodeLastTwoBits(packet.highg_ax, packet.highg_ay, packet.highg_az);
    int16_t ax = packet.highg_ax & 0xfffc;
    int16_t ay = packet.highg_ay & 0xfffc;
    int16_t az = packet.highg_az & 0xfffc;
    data.highG_ax = convert_range<int16_t>(ax, 32);
    data.highG_ay = convert_range<int16_t>(ay, 32);
    data.highG_az = convert_range<int16_t>(az, 32);
    data.tilt_angle = tilt; //convert_range(tilt, 180); // [-90, 90]
    data.battery_voltage = convert_range(packet.batt_volt, 16);
    data.sat_count = packet.fsm_satcount >> 4;
    data.FSM_State = packet.fsm_satcount & 0b1111;
    data.freq = RF95_FREQ;
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
    Serial.print(R"({"type": "data", "value": {)");
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
    printJSONField("sat_count", packet.sat_count, false);
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
    #ifdef IS_GROUND
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println(json_set_frequency_failure);
        while (1);
    }
    #endif
    #ifdef IS_DRONE
    if (!rf95.setFrequency(SUSTAINER_FREQ)) {
        Serial.println(json_set_frequency_failure);
        current_freq = SUSTAINER_FREQ;
        while (1);

    }
    #endif
    rf95.setCodingRate4(8);
    rf95.setSpreadingFactor(10);
    rf95.setPayloadCRC(true);
    rf95.setSignalBandwidth(125000);
    Serial.print(R"({"type": "freq_success", "frequency":)");
    Serial.print(RF95_FREQ);
    Serial.println("}");
    rf95.setTxPower(23, false);
}

void ChangeFrequency(float freq) {
    rf95.setFrequency(freq);
    Serial.println(json_command_success);
    Serial.print(R"({"type": "freq_success", "frequency":)");
    Serial.print(freq);
    Serial.println("}");
}

#ifdef IS_GROUND

void loop() {
    
    PrintDequeue();
    if (rf95.available()) {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        TelemetryPacket packet;
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(50);
            digitalWrite(LED_BUILTIN, LOW);
            // Serial.println("Received packet");
            // Serial.println(len);
            memcpy(&packet, buf, sizeof(packet));
            EnqueuePacket(packet, current_freq);
            if (!cmd_queue.empty()) {
                auto& cmd = cmd_queue.front();
                    cmd.retry_count++;
                    if (cmd.retry_count >= max_command_retries) {
                        cmd_queue.pop();
                        Serial.println(json_send_failure);
                    }
            }

            process_command_queue();

        } else {
            Serial.println(json_receive_failure);
        }
    }
    serial_parser.read();
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        if (input.startsWith("FREQ:")) {
            float freq = input.substring(5).toFloat(); // Extract frequency value
            ChangeFrequency(freq);
        }
    }
}
#endif



#ifdef IS_DRONE

void loop() {
    
    PrintDequeue();
    TelemetryPacket packet;

    if (rf95.available()) {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(50);
            digitalWrite(LED_BUILTIN, LOW);
            ChangeFrequency(GROUND_FREQ);
            memcpy(&packet, buf, sizeof(packet));
            if(current_freq == SUSTAINER_FREQ) {
                packet.IS_SUSTAINER = true;
            } else {
                packet.IS_SUSTAINER = false;
            }
            uint8_t packetBuffer[sizeof(packet)];
            memcpy(packetBuffer, &packet, sizeof(packet));
            rf95.send(packetBuffer, len);
            if(current_freq == SUSTAINER_FREQ) {
                ChangeFrequency(BOOSTER_FREQ);
            } else {
                ChangeFrequency(SUSTAINER_FREQ);
            }
        } else {
            Serial.println(json_receive_failure);
        }
    }
    serial_parser.read();
}
#endif