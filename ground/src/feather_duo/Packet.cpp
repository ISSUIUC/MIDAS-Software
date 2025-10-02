#include<Arduino.h>
#include"Packet.h"


int decodeLastTwoBits(uint16_t ax, uint16_t ay, uint16_t az) {
    int tilt_ax = ax & 0b11;
    int tilt_ay = ay & 0b11;
    int tilt_az = az & 0b11;
    int tilt = (tilt_ax << 0) | (tilt_ay << 2) | (tilt_az << 4);
    return tilt;
}

double ConvertGPS(int32_t coord) {
    double complete = static_cast<double>(coord) / 10000000.0;
    return complete;
}

template <typename T>
float convert_range(T val, float range) {
    size_t numeric_range = (int64_t)std::numeric_limits<T>::max() - (int64_t)std::numeric_limits<T>::min() + 1;
    return val * range / (float)numeric_range;
}

FullTelemetryData DecodePacket(const TelemetryPacket& packet, float frequency) {
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

    constexpr float max_roll_rate_hz = 10.0f;

    data.pyros[0] = ((float) ((packet.pyro >> 0) & (0xFF)) / 255) * max_roll_rate_hz; // Pyro A is rotation rate
    data.pyros[1] = ((float) ((packet.pyro >> 8) & (0xFF)));                          // Pyro B is camera state
    data.pyros[2] = ((float) ((packet.pyro >> 16) & (0xFFF)) / 4095.) * 40000.;       // Pyro C is kf_px
    data.pyros[3] = ((float) ((packet.pyro >> 21) & (0x7F)) / 127.) * 12.;

    data.kf_reset = packet.alt & 1;

    // kinda hacky but it will work
    if (packet.fsm_callsign_satcount == static_cast<uint8_t>(-1)) {
        data.FSM_State = static_cast<uint8_t>(-1);
    }
    data.kf_vx = (float) packet.kf_vx / (float) ((1 << 16) - 1) * 4000.f - 2000.f;
    data.freq = frequency;

    return data;
}

DecodedGNCData DecodePacketGNC(const GNCTelemData& packet, float frequency) {
    int64_t start_printing = millis();
    DecodedGNCData data;

    data.k_pos_x = convert_range<int16_t>(packet.k_pos_x, 10000);
    data.k_pos_y = convert_range<int16_t>(packet.k_pos_y, 10000);
    data.k_pos_z = convert_range<int16_t>(packet.k_pos_z, 10000);

    data.k_vel_x = convert_range<int16_t>(packet.k_vel_x, 1000);
    data.k_vel_y = convert_range<int16_t>(packet.k_vel_y, 1000);
    data.k_vel_z = convert_range<int16_t>(packet.k_vel_z, 1000);

    data.k_acc_x = convert_range<int16_t>(packet.k_acc_x, 1000);
    data.k_acc_y = convert_range<int16_t>(packet.k_acc_y, 1000);
    data.k_acc_z = convert_range<int16_t>(packet.k_acc_z, 1000);

    data.k_altitude = convert_range<int16_t>(packet.k_altitude, 10000);

    // Decode sensor readings
    data.r_ax = convert_range<int16_t>(packet.r_ax, 32);
    data.r_ay = convert_range<int16_t>(packet.r_ay, 32);
    data.r_az = convert_range<int16_t>(packet.r_az, 32);

    data.r_pitch = convert_range<int16_t>(packet.r_pitch, 100);
    data.r_roll = convert_range<int16_t>(packet.r_roll, 100);
    data.r_yaw = convert_range<int16_t>(packet.r_yaw, 100);

    data.r_tilt = packet.r_tilt;

    data.kf_reset = (packet.fsm_callsign_ack >> 4) & 1;
    data.FSM_State = packet.fsm_callsign_ack & 0b1111;
    data.is_sustainer = (packet.fsm_callsign_ack >> 5);


    if (packet.fsm_callsign_ack == static_cast<uint8_t>(-1)) {
        data.FSM_State = static_cast<uint8_t>(-1);
    }

    data.freq = frequency;

    return data;
}