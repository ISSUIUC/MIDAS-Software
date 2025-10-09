#include <cmath>

#include "telemetry.h"

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//fsm state not working
//check gps
//check barom
//tilt angle not working but roll rate is fine
//pyros not working
//

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

    packet.alt = (((int16_t) gps.altitude) & 0xfffe) | (received_count & 0x0001);    // Convert range of value so that we can also account for negative altitudes (converting float to 15 bit alt, 1 bit command ack)
    
    
    packet.baro_alt = inv_convert_range<int16_t>(barometer.altitude, 1 << 17);
  
    //auto [ax,ay,az, tilt_extra] = pack_highg_tilt(highg, map(static_cast<long>(orientation.tilt * 100),0, 314, 0, 1023));
    packet.highg_ax = (uint16_t)inv_convert_range<int16_t>(highg.ax, MAX_ABS_ACCEL_RANGE_G);
    packet.highg_ay = (uint16_t)inv_convert_range<int16_t>(highg.ay, MAX_ABS_ACCEL_RANGE_G);
    packet.highg_az = (uint16_t)inv_convert_range<int16_t>(highg.az, MAX_ABS_ACCEL_RANGE_G);

    // Tilt & FSM State
    static_assert(FSMState::FSM_STATE_COUNT < 16);
    uint16_t tilt_norm = (orientation.tilt / M_PI) * 0x0fff; // Encodes tilt value 0-1 into range 0x0000 - 0x0fff
    packet.tilt_fsm |= ((tilt_norm << 4) & 0xfff0);
    packet.tilt_fsm |= ((uint16_t)fsm & 0x000f);

    //Serial.println(packet.tilt_fsm, 2);
    // Battery voltage
    packet.batt_volt = inv_convert_range<uint8_t>(voltage.voltage, MAX_TELEM_VOLTAGE_V);
    
    // Roll rate
    float roll_rate_hz = std::clamp(std::abs(orientation.angular_velocity.vx) / (2.0f*static_cast<float>(PI)), 0.0f, MAX_ROLL_RATE_HZ);
    packet.roll_rate = roll_rate_hz / MAX_ROLL_RATE_HZ * 0xFF;

    // KF data
    packet.kf_px = inv_convert_range<int16_t>(kalman.position.px, MAX_KF_XPOSITION_M);

    Serial.println(kalman.position.px);

    packet.kf_vx = inv_convert_range<int16_t>(kalman.velocity.vx, MAX_KF_XVELOCITY_MS);

    //Camera Data
    packet.camera_state = ((uint16_t) (data.camera_state)) & 0xFF;
    
    //Pyro A0 | B1 | C2 | D3
    // This is what we're telemetering for MIDAS mk2
    packet.pyro |= ((((uint32_t) (std::round(continuity.pins[0]))) & 0x7F) << (0 * 7));
    packet.pyro |= ((((uint32_t) (continuity.pins[1] / MAX_TELEM_VOLTAGE_V * 127)) & 0x7F) << (1 * 7));
    packet.pyro |= ((((uint32_t) (continuity.pins[2] / MAX_TELEM_VOLTAGE_V * 127)) & 0x7F) << (2 * 7));
    packet.pyro |= ((((uint32_t) (continuity.pins[3] / MAX_TELEM_VOLTAGE_V * 127)) & 0x7F) << (3 * 7));

    // This is what we want for MIDAS mk3
    // packet.pyro |= (uint8_t)inv_convert_range<int8_t>(pins[0], MAX_TELEM_VOLTAGE_V);
    // packet.pyro |= ((uint32_t)inv_convert_range<int8_t>(pins[1], (float)std::numeric_limits<float>::max()) & 0xFF) << (1*8);
    // packet.pyro |= ((uint32_t)inv_convert_range<int8_t>(pins[2], (float)std::numeric_limits<float>::max()) & 0xFF) << (2*8);
    // packet.pyro |= ((uint32_t)inv_convert_range<int8_t>(pins[3], (float)std::numeric_limits<float>::max()) & 0xFF) << (3*8);

    // GPS state & Callsign
    // 0000 | 000 | 0
    // SATC | FT  | C
    packet.callsign_gpsfix_satcount |= (gps.fix_type & 0x07) << 1;
    packet.callsign_gpsfix_satcount |= (gps.fix_type & 0x0F) << 4; // Replace fix type here with satcount later

    #ifdef IS_SUSTAINER
    packet.callsign_gpsfix_satcount |= 0b1;
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
