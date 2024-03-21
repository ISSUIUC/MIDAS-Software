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


void Telemetry::transmit(RocketData& rocket_data, LEDController& led) {
//    telemetry_command command { };
//    while (backend.read(&command)) {
//        handleCommand(command);
//    }

//    if (!std::isnan(set_frequency_to)) {
//        backend.setFrequency(set_frequency_to);
//        set_frequency_to = NAN;
//    }

    TelemetryPacket packet = makePacket(rocket_data);
    led.toggle(LED::BLUE);
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
        size_t callsign_size = sizeof(callsign);

        for (size_t i = 0; i < callsign_size; ++i) {
            callsign[i] = cmd.callsign[i];
        }
        Serial.println("[DEBUG]: Got callsign");
    }
}


TelemetryPacket Telemetry::makePacket(RocketData& data) {
    TelemetryPacket packet { };

    TelemetryDataLite small_packet { };
    packet.datapoint_count = 0;
    for (int8_t i = 0; i < 4 && small_packet_queue.receive(&small_packet); i++) {
        packet.datapoints[i] = small_packet;
        packet.datapoint_count++;
    }

    GPS gps = data.gps.getRecentUnsync();
    Magnetometer magnetometer = data.magnetometer.getRecentUnsync();
    Voltage voltage = data.voltage.getRecentUnsync();
    packet.voltage_battery = inv_convert_range<uint16_t>(voltage.voltage, 4096);
    Barometer barometer = data.barometer.getRecentUnsync();
    LowGLSM lowGlsm = data.low_g_lsm.getRecentUnsync();

    packet.gps_lat = gps.latitude;
    packet.gps_long = gps.longitude;
    packet.gps_alt = gps.altitude;

    packet.mag_x = inv_convert_range<int16_t>(magnetometer.mx, 8);
    packet.mag_y = inv_convert_range<int16_t>(magnetometer.my, 8);
    packet.mag_z = inv_convert_range<int16_t>(magnetometer.mz, 8);
    packet.gyro_x = inv_convert_range<int16_t>(lowGlsm.gx, 8192);
    packet.gyro_y = inv_convert_range<int16_t>(lowGlsm.gy, 8192);
    packet.gyro_z = inv_convert_range<int16_t>(lowGlsm.gz, 8192);

    packet.rssi = backend.getRecentRssi();
    packet.FSM_state = (char) data.fsm_state.getRecentUnsync();

    packet.barometer_temp = inv_convert_range<int16_t>(barometer.temperature, 256);

    auto pyros = data.pyro.getRecentUnsync();
    packet.pyros_bits = 0;
    packet.pyros_bits |= pyros.channels[0].is_armed << 0;
    packet.pyros_bits |= pyros.channels[1].is_armed << 1;
    packet.pyros_bits |= pyros.channels[2].is_armed << 2;
    packet.pyros_bits |= pyros.channels[3].is_armed << 3;
    packet.pyros_bits |= pyros.channels[0].is_firing << 4;
    packet.pyros_bits |= pyros.channels[1].is_firing << 5;
    packet.pyros_bits |= pyros.channels[2].is_firing << 6;
    packet.pyros_bits |= pyros.channels[3].is_firing << 7;

    Continuity continuity = data.continuity.getRecentUnsync();
    for (int i = 0; i < 4; i++) {
       packet.continuity[i] = inv_convert_range<int8_t>(continuity.pins[i], 20);
    }

    packet.telem_latency = inv_convert_range<int8_t>((float) data.telem_latency.getLatency(), 1024);
    packet.log_latency = inv_convert_range<int8_t>((float) data.log_latency.getLatency(), 1024);

#ifdef IS_BOOSTER
    packet.is_booster = true;
#else
    packet.is_booster = false;
#endif

    memcpy(&packet.callsign, &callsign, sizeof(callsign));

    return packet;
}

void Telemetry::bufferData(RocketData& rocket) {
    TelemetryDataLite data { };
    data.timestamp = pdTICKS_TO_MS(xTaskGetTickCount());
    data.barometer_pressure = inv_convert_range<uint16_t>(rocket.barometer.getRecentUnsync().pressure , 4096);

    HighGData highGData = rocket.high_g.getRecentUnsync();
    data.highG_ax = inv_convert_range<int16_t>(highGData.ax, 256);
    data.highG_ay = inv_convert_range<int16_t>(highGData.ay, 256);
    data.highG_az = inv_convert_range<int16_t>(highGData.az, 256);

    LowGLSM lowGlsm = rocket.low_g_lsm.getRecentUnsync();
    data.lowg_ax = inv_convert_range<int16_t>(lowGlsm.ax, 8);
    data.lowg_ay = inv_convert_range<int16_t>(lowGlsm.ay, 8);
    data.lowg_az = inv_convert_range<int16_t>(lowGlsm.az, 8);

    Orientation orient = rocket.orientation.getRecentUnsync();
    data.bno_pitch = inv_convert_range<int16_t>(orient.pitch , 8);
    data.bno_yaw = inv_convert_range<int16_t>(orient.yaw , 8);
    data.bno_roll = inv_convert_range<int16_t>(orient.roll , 8);

    small_packet_queue.send(data);
}

ErrorCode __attribute__((warn_unused_result)) Telemetry::init() {
    return backend.init();
}
