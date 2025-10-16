#include"Output.h"
#include<Arduino.h>
#include <algorithm>

void printFloat(float f, int precision = 5) {
    if (isinf(f) || isnan(f)) {
        Serial.print(-1);
    } else {
        Serial.print(f, precision);
    }
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
    char buff[1024]{};
    int len = sprintf(buff, R"({"type": "data", "value": {"barometer_altitude": %f, "latitude": %f, "longitude": %f, "altitude": %i, "highG_ax": %f, "highG_ay": %f, "highG_az": %f, "battery_voltage": %f, "FSM_State": %i, "tilt_angle": %f, "frequency": %f, "RSSI": %f, "sat_count": %f, "kf_velocity": %f, "kf_position": %f, "is_sustainer": %i, "roll_rate": %f, "c_valid": %u, "c_on": %u, "c_rec": %u, "vtx_on": %u, "vmux_stat": %u, "cam_ack": %u, "kf_reset": %i}})",
    packet.barometer_altitude,
    packet.latitude,
    packet.longitude,
    packet.altitude,
    packet.highG_ax,
    packet.highG_ay,
    packet.highG_az,
    packet.battery_voltage,
    packet.FSM_State,
    packet.tilt_angle,
    packet.freq,
    packet.rssi,
    packet.sat_count,
    packet.kf_vx,
    packet.pyros[2],
    packet.is_sustainer,
    packet.pyros[0],
    ((((uint8_t) round(packet.pyros[1])) >> 7) & 0x01 ), // c_valid
    (((uint8_t) round(packet.pyros[1])) >> 0) & 0x03, // c_on
    (((uint8_t) round(packet.pyros[1])) >> 2) & 0x03, // c_rec
    (((uint8_t) round(packet.pyros[1])) >> 4) & 0x01, //vtx_on
    (((uint8_t) round(packet.pyros[1])) >> 5) & 0x01, //vmux_stat
    (((uint8_t) round(packet.pyros[1])) >> 6) & 0x01, //cam_ack
    packet.kf_reset
    );
    Serial.println(buff);
    // Serial.print(R"({"type": ")");
    // Serial.print(is_heartbeat ? "heartbeat" : "data");
    // Serial.print(R"(", "value": {)");
    // printJSONField("barometer_altitude", packet.barometer_altitude);
    // printJSONField("latitude", packet.latitude);
    // printJSONField("longitude", packet.longitude);
    // printJSONField("altitude", packet.altitude);
    // printJSONField("highG_ax", packet.highG_ax);
    // printJSONField("highG_ay", packet.highG_ay);
    // printJSONField("highG_az", packet.highG_az);
    // printJSONField("battery_voltage", packet.battery_voltage);
    // printJSONField("FSM_State", packet.FSM_State);
    // printJSONField("tilt_angle", packet.tilt_angle);
    // printJSONField("frequency", packet.freq);
    // printJSONField("RSSI", packet.rssi);
    // printJSONField("sat_count", packet.sat_count);
    // printJSONField("kf_velocity", packet.kf_vx);
    // printJSONField("is_sustainer", packet.is_sustainer);
    // printJSONField("pyro_a", packet.pyros[0]);
    // printJSONField("pyro_b", packet.pyros[1]);
    // printJSONField("pyro_c", packet.pyros[2]);
    // printJSONField("kf_reset", packet.kf_reset);
    // printJSONField("pyro_d", packet.pyros[3], false);
    // Serial.println("}}");
}

void printDronePacketJson(FullTelemetryData const& packet) {
    bool is_heartbeat = packet.FSM_State == static_cast<uint8_t>(-1);
    char buff[1024]{}; // Leaving the same buffer size as PacketJson
    
    int len = sprintf(buff, R"({"type": "data", "value": 
        {"barometer_altitude": %f, "latitude": %f, 
        "longitude": %f, "altitude": %i, 
        "FSM_State": %i, "frequency": %f, 
        "RSSI": %f, "sat_count": %f, 
        "is_sustainer": %i, "kf_reset": %i}})",

    packet.barometer_altitude,
    packet.latitude,
    packet.longitude,
    packet.altitude,
    packet.FSM_State,
    packet.freq,
    packet.rssi,
    packet.sat_count,
    packet.is_sustainer,
    packet.kf_reset
    );
    Serial.println(buff);
}
