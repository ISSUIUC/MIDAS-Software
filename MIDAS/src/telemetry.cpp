#include <cmath>

#include "telemetry.h"


/**
 * @brief This function maps an input value onto within a particular range into a fixed point value of a certin binary
 * size
 *
 * @param val: number to map into target range, values outside of the range will be clamped
 *
 * @param range: range to map number into. For unsigned output, [0, range). For signed output [-range/2, range)
 *
 * @return fixed point value represing val mapped onto the target range
 */
template <typename T>
T inv_convert_range(float val, float range) {
    size_t numeric_range = (int64_t)std::numeric_limits<T>::max() - (int64_t)std::numeric_limits<T>::min() + 1;
    float converted = val * (float)numeric_range / range;
    return std::max(std::min((float)std::numeric_limits<T>::max(), converted), (float)std::numeric_limits<T>::min());
}


Telemetry::Telemetry(TelemetryBackend&& backend) : backend(std::move(backend)) { }


void Telemetry::transmit(RocketData& rocket_data) {
    telemetry_command command { };
    while (backend.read(&command)) {
        handleCommand(command);
    }

    if (!std::isnan(set_frequency_to)) {
        backend.setFrequency(set_frequency_to);
        set_frequency_to = NAN;
    }

    TelemetryPacket packet = makePacket(rocket_data);
    backend.send(packet);
}


/**
 * @brief  This function handles commands sent from the ground station
 * to TARS. The effects of this function depend on the command
 * sent.
 *
 * @param cmd: struct containing information necessary to process
 *             ground station command.
 *
 * @return void
 */
void Telemetry::handleCommand(const telemetry_command &cmd) {
    /* Check if the security code is present and matches on ground and on the rocket */
    if (cmd.verify != std::array<char, 6>{'A', 'Y', 'B', 'E', 'R', 'K'}) {
        return;
    }
    /* Check if lasted command ID matched current command ID */
    if (last_command_id == cmd.cmd_id) {
        return;
    }
    last_command_id = (int16_t)cmd.cmd_id;

    if (cmd.command == SET_FREQ) {
        set_frequency_to = cmd.freq;
    }

    if (cmd.command == SET_CALLSIGN) {
        memcpy(callsign, cmd.callsign, sizeof(cmd.callsign));
        Serial.println("[DEBUG]: Got callsign");
    }
}


TelemetryPacket Telemetry::makePacket(RocketData& data) {
    TelemetryPacket packet { };
    packet.gps_lat = data.gps.getRecent().latitude;
    packet.gps_long = data.gps.getRecent().longitude;
    packet.gps_alt = data.gps.getRecent().altitude;
    packet.yaw = data.orientation.getRecent().yaw;
    packet.pitch = data.orientation.getRecent().pitch;
    packet.roll = data.orientation.getRecent().roll;

    packet.mag_x = inv_convert_range<int16_t>(data.magnetometer.getRecent().mx, 8);
    packet.mag_y = inv_convert_range<int16_t>(data.magnetometer.getRecent().my, 8);
    packet.mag_z = inv_convert_range<int16_t>(data.magnetometer.getRecent().mz, 8);
    packet.gyro_x = inv_convert_range<int16_t>(data.orientation.getRecent().gx, 8192);
    packet.gyro_y = inv_convert_range<int16_t>(data.orientation.getRecent().gy, 8192);
    packet.gyro_z = inv_convert_range<int16_t>(data.orientation.getRecent().gz, 8192);

    packet.response_ID = last_command_id;

    packet.rssi = backend.getRecentRssi();
    packet.voltage_battery = inv_convert_range<uint8_t>(data.voltage.getRecent().voltage , 16);
    packet.barometer_temp = inv_convert_range<int16_t>(data.barometer.getRecent().temperature, 256);

    TelemetryDataLite small_packet { };
    packet.datapoint_count = 0;
    for (int8_t i = 0; i < 4 && small_packet_queue.receive(&small_packet); i++) {
        packet.datapoints[i] = small_packet;
        packet.datapoint_count = i + 1;
    }
    return packet;
}

void Telemetry::bufferData(RocketData& Sensorstate) {
    TelemetryDataLite data { };
    data.timestamp = pdTICKS_TO_MS(xTaskGetTickCount());
    data.barometer_pressure = inv_convert_range<uint16_t>(Sensorstate.barometer.getRecent().pressure , 4096);

    data.highG_ax = inv_convert_range<int16_t>(Sensorstate.high_g.getRecent().ax, 256);
    data.highG_ay = inv_convert_range<int16_t>(Sensorstate.high_g.getRecent().ay , 256);
    data.highG_az = inv_convert_range<int16_t>(Sensorstate.high_g.getRecent().az , 256);

    data.bno_pitch = inv_convert_range<int16_t>(Sensorstate.orientation.getRecent().pitch , 8);
    data.bno_yaw = inv_convert_range<int16_t>(Sensorstate.orientation.getRecent().yaw , 8);
    data.bno_roll = inv_convert_range<int16_t>(Sensorstate.orientation.getRecent().roll , 8);

    small_packet_queue.send(data);
}

ErrorCode __attribute__((warn_unused_result)) Telemetry::init() {
    return backend.init();
}
