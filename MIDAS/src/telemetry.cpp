#include <cmath>

#include "telemetry.h"

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


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

/**
 * @brief packs highg and tilt infomation into 3, 2 byte integers
 * 
 * @param highg highg data to store
 * @param tilt tilt information to store
 * 
 * @return tuple with packed data
*/
std::tuple<uint16_t, uint16_t, uint16_t, uint16_t> pack_highg_tilt(HighGData const& highg, uint16_t tilt) {
    uint16_t ax = (uint16_t)inv_convert_range<int16_t>(highg.ax, 32);
    uint16_t ay = (uint16_t)inv_convert_range<int16_t>(highg.ay, 32);
    uint16_t az = (uint16_t)inv_convert_range<int16_t>(highg.az, 32);

    uint16_t x = (ax & 0xfffc) | ((tilt >> 0) & 0x3);
    uint16_t y = (ay & 0xfffc) | ((tilt >> 2) & 0x3);
    uint16_t z = (az & 0xfffc) | ((tilt >> 4) & 0x3);
    uint16_t q = (tilt >> 6) & 15;

    return {x,y,z,q};
}

/**
 * @brief move constructor for the telemetry backend
*/
Telemetry::Telemetry(TelemetryBackend&& backend) : backend(std::move(backend)) { }

/**
 * @brief transmit telemetry data through LoRa
 * 
 * @param rocket_data rocket_data to transmit
 * @param led led state to transmit
*/
void Telemetry::transmit(RocketData& rocket_data, LEDController& led) {
    // static_assert(sizeof(TelemetryPacket) == 20);

    TelemetryPacket packet = makePacket(rocket_data);
    led.toggle(LED::BLUE);

    backend.send(packet);
}

bool Telemetry::receive(TelemetryCommand* command, int wait_milliseconds) {
    return backend.read(command, wait_milliseconds);
}

void Telemetry::acknowledgeReceived() {
    received_count ++;
}

/**
 * @brief creates the packet to send through the telemetry system
 * 
 * @param data the data to serialize into a packet
*/
TelemetryPacket Telemetry::makePacket(RocketData& data) {

    TelemetryPacket packet { };
    GPS gps = data.gps.getRecentUnsync();
    Voltage voltage = data.voltage.getRecentUnsync();
    Barometer barometer = data.barometer.getRecentUnsync();
    FSMState fsm = data.fsm_state.getRecentUnsync();
    Continuity continuity = data.continuity.getRecentUnsync();
    HighGData highg = data.high_g.getRecentUnsync();
    PyroState pyro = data.pyro.getRecentUnsync();
    Orientation orientation = data.orientation.getRecentUnsync();
    KalmanData kalman = data.kalman.getRecentUnsync();

    packet.lat = gps.latitude;
    packet.lon = gps.longitude;
    packet.alt = (((int16_t) gps.altitude) & 0xfffe) | (received_count & 0x0001);    // Convert range of value so that we can also account for negative altitudes
    packet.baro_alt = inv_convert_range<int16_t>(barometer.altitude, 1 << 17);
  
    auto [ax,ay,az, tilt_extra] = pack_highg_tilt(highg, map(static_cast<long>(orientation.tilt * 100),0, 314, 0, 1023));
    packet.highg_ax = ax;
    packet.highg_ay = ay;
    packet.highg_az = az;
    packet.batt_volt = inv_convert_range<uint8_t>(voltage.voltage, 16);
    
    const float max_volts = 12;
    packet.pyro |= ((((uint16_t) (continuity.pins[0] / max_volts * 127)) & 0x7F) << (0 * 7));
    packet.pyro |= ((((uint16_t) (continuity.pins[1] / max_volts * 127)) & 0x7F) << (1 * 7));
    packet.pyro |= ((((uint16_t) (continuity.pins[2] / max_volts * 127)) & 0x7F) << (2 * 7));
    packet.pyro |= ((((uint16_t) (continuity.pins[3] / max_volts * 127)) & 0x7F) << (3 * 7));
    packet.pyro |= tilt_extra << 28;

    static_assert(FSMState::FSM_STATE_COUNT < 16);
    uint8_t sat_count = gps.fix_type;
    packet.fsm_callsign_satcount = ((uint8_t)fsm) | (sat_count << 4);
    float kf_vx_clamped = std::clamp(kalman.velocity.vx, -2000.f, 2000.f);
    packet.kf_vx = (uint16_t) ((kf_vx_clamped + 2000) / 4000. * ((1 << 16) - 1));

    #ifdef IS_SUSTAINER
    packet.fsm_callsign_satcount |= (1 << 7);
    #endif

    return packet;
}

/**
 * @brief initializes the Telemetry system
 * 
 * @return Error Code
*/
ErrorCode __attribute__((warn_unused_result)) Telemetry::init() {
    return backend.init();
}
