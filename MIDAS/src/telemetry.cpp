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
    static_assert(sizeof(TelemetryPacket) == 20);

    TelemetryPacket packet = makePacket(rocket_data);
    led.toggle(LED::BLUE);
    backend.send(packet);
}

TelemetryPacket Telemetry::makePacket(RocketData& data) {
    TelemetryPacket packet { };
    GPS gps = data.gps.getRecentUnsync();
    Voltage voltage = data.voltage.getRecentUnsync();
    Barometer barometer = data.barometer.getRecentUnsync();
    FSMState fsm = data.fsm_state.getRecentUnsync();
    Continuity continuity = data.continuity.getRecentUnsync();
    HighGData highg = data.high_g.getRecentUnsync();

    packet.alt = uint16_t(gps.altitude);
    packet.highg_ax = inv_convert_range<int16_t>(highg.ax,128);
    packet.highg_ay = inv_convert_range<int16_t>(highg.ay,128);
    packet.highg_az = inv_convert_range<int16_t>(highg.az,128);
    packet.baro_alt = uint16_t(barometer.altitude);

    packet.batt_volt

    return packet;
}

ErrorCode __attribute__((warn_unused_result)) Telemetry::init() {
    return backend.init();
}
