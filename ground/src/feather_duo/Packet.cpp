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

    // GPS
    data.altitude = static_cast<float>(packet.alt & 0xfffe); // Only convert top 15 bits
    data.latitude = ConvertGPS(packet.lat);
    data.longitude = ConvertGPS(packet.lon);
    data.cmd_ack = packet.alt & 1; // Ack bit

    // Barometer
    data.barometer_altitude = convert_range<int16_t>(packet.baro_alt, 1 << 17);

    // Tilt & FSM
    data.tilt_angle = ((float)((packet.tilt_fsm >> 4) & 0x0fff) / 0x0fff) * 180;
    data.FSM_State = packet.tilt_fsm & 0x000f;

    // Acceleration
    int16_t ax = packet.highg_ax;
    int16_t ay = packet.highg_ay;
    int16_t az = packet.highg_az;
    
    data.highG_ax = convert_range<int16_t>(ax, MAX_ABS_ACCEL_RANGE_G);
    data.highG_ay = convert_range<int16_t>(ay, MAX_ABS_ACCEL_RANGE_G);
    data.highG_az = convert_range<int16_t>(az, MAX_ABS_ACCEL_RANGE_G);

    // Other data
    data.battery_voltage = convert_range(packet.batt_volt, MAX_TELEM_VOLTAGE_V);
    data.gps_fixtype = packet.callsign_gpsfix_satcount >> 1 & 0b0111;
    data.is_sustainer = (packet.callsign_gpsfix_satcount & 0b1);
    data.sat_count = (packet.callsign_gpsfix_satcount >> 4) & 0b1111;

    //Pyros
    data.pyros[0] = (packet.pyro >> 0) & (0x7F);
    data.pyros[1] = ((float) ((packet.pyro >> 7) & (0x7F)) / 127.) * MAX_TELEM_VOLTAGE_V;
    data.pyros[2] = ((float) ((packet.pyro >> 14) & (0x7F)) / 127.) * MAX_TELEM_VOLTAGE_V;
    data.pyros[3] = ((float) ((packet.pyro >> 21) & (0x7F)) / 127.) * MAX_TELEM_VOLTAGE_V;

    // TODO: MIDAS MK3 will have proper pyro telem reporting.
    
    // data.pyros[0] = (packet.pyro >> 0) & (0x7F);
    // data.pyros[1] = ((float) ((packet.pyro >> 7) & (0x7F)) / 127.) * MAX_TELEM_VOLTAGE_V;
    // data.pyros[2] = ((float) ((packet.pyro >> 14) & (0x7F)) / 127.) * MAX_TELEM_VOLTAGE_V;
    // data.pyros[3] = ((float) ((packet.pyro >> 21) & (0x7F)) / 127.) * MAX_TELEM_VOLTAGE_V;

    // kalman filter    
    data.kf_px = convert_range<uint16_t>(packet.kf_px, MAX_KF_XPOSITION_M);
    data.kf_vx = convert_range<int16_t>(packet.kf_vx, MAX_KF_XVELOCITY_MS);

    // Camera state
    data.camera_state = packet.camera_state;

    // Roll rate
    data.roll_rate_hz = ((float)packet.roll_rate / 0xFF) * MAX_ROLL_RATE_HZ;


    data.freq = frequency;

    return data;
}