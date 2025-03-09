#include"Output.h"
#include<Arduino.h>

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
