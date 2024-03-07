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
//    telemetry_command command { };
//    while (backend.read(&command)) {
//        handleCommand(command);
//    }

//    if (!std::isnan(set_frequency_to)) {
//        backend.setFrequency(set_frequency_to);
//        set_frequency_to = NAN;
//    }

    Serial.println("Making Packet...");  Serial.flush();
    TelemetryPacket packet = makePacket(rocket_data);
    Serial.println("Sending Packet...");  Serial.flush();
//    char packet[200] = "moooooo";

    backend.send(packet);

    Serial.println("Sent Packet..."); Serial.flush();
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
        size_t callsign_size = sizeof(callsign);

        for (size_t i = 0; i < callsign_size; ++i) {
            callsign[i] = cmd.callsign[i];
        }
        Serial.println("[DEBUG]: Got callsign");
    }
}


TelemetryPacket Telemetry::makePacket(RocketData& data) {
    TelemetryPacket packet { };


    Serial.println("Making Buffered packets"); Serial.flush();
    TelemetryDataLite small_packet { };
    packet.datapoint_count = 0;
    for (int8_t i = 0; i < 4 && small_packet_queue.receive(&small_packet); i++) {
        packet.datapoints[i] = small_packet;
        packet.datapoint_count = i + 1;
    }


//    GPS gps = data.gps.getRecent();
//    Orientation orientation = data.orientation.getRecent();
//    Magnetometer magnetometer = data.magnetometer.getRecent();
//    Voltage voltage = data.voltage.getRecent();
//    packet.voltage_battery = inv_convert_range<uint8_t>(voltage.voltage , 16);
//    Barometer barometer = data.barometer.getRecent();

//    packet.gps_lat = gps.latitude;
//    packet.gps_long = gps.longitude;
//    packet.gps_alt = gps.altitude;
//    packet.yaw = orientation.yaw;
//    packet.pitch = orientation.pitch;
//    packet.roll = orientation.roll;

//    packet.mag_x = inv_convert_range<int16_t>(magnetometer.mx, 8);
//    packet.mag_y = inv_convert_range<int16_t>(magnetometer.my, 8);
//    packet.mag_z = inv_convert_range<int16_t>(magnetometer.mz, 8);
//    packet.gyro_x = inv_convert_range<int16_t>(orientation.gx, 8192);
//    packet.gyro_y = inv_convert_range<int16_t>(orientation.gy, 8192);
//    packet.gyro_z = inv_convert_range<int16_t>(orientation.gz, 8192);

    packet.response_ID = last_command_id;

    packet.rssi = backend.getRecentRssi();
    Serial.println("Getting fsm"); Serial.flush();
    packet.FSM_state = (char) data.fsm_state.getRecent();

    Serial.println("Getting pyros"); Serial.flush();

//    packet.barometer_temp = inv_convert_range<int16_t>(barometer.temperature, 256);

    auto pyros = data.pyro.getRecent();
    for (int i = 0; i < 4; i++) {
        packet.pyros_armed[i] = pyros.channels[i].is_armed;
        packet.pyros_firing[i] = pyros.channels[i].is_firing;
    }

//    Continuity continuity{};
//    data.continuity.getRecent2(&continuity);
//    Continuity continuity = data.continuity.getRecent();
//    for (int i = 0; i < 4; i++) {
//        packet.continuity[i] = continuity.pins[i];
//    }

    Serial.println("Memcpy'ing"); Serial.flush();

    memcpy(&packet.callsign, &callsign, sizeof(callsign));

    Serial.println("Done with packet"); Serial.flush();

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
