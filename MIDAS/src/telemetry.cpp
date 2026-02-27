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
// std::tuple<uint16_t, uint16_t, uint16_t, uint16_t> pack_highg_tilt(HighGData const& highg, uint16_t tilt) { //update parameter and function
    
//     uint16_t ax = (uint16_t)inv_convert_range<int16_t>(highg.ax, 32);
//     uint16_t ay = (uint16_t)inv_convert_range<int16_t>(highg.ay, 32);
//     uint16_t az = (uint16_t)inv_convert_range<int16_t>(highg.az, 32);

//     uint16_t x = (ax & 0xfffc) | ((tilt >> 0) & 0x3);
//     uint16_t y = (ay & 0xfffc) | ((tilt >> 2) & 0x3);
//     uint16_t z = (az & 0xfffc) | ((tilt >> 4) & 0x3);
//     uint16_t q = (tilt >> 6) & 15;

//     return {x,y,z,q};
// }

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
    IMU imu = data.imu.getRecentUnsync();
    IMU_SFLP hw_filtered_data = data.hw_filtered.getRecentUnsync();
    GPS gps = data.gps.getRecentUnsync();
    Voltage voltage = data.voltage.getRecentUnsync();
    Barometer barometer = data.barometer.getRecentUnsync();
    FSMState fsm = data.fsm_state.getRecentUnsync();
    PyroState pyro = data.pyro.getRecentUnsync();
    KalmanData kalman = data.kalman.getRecentUnsync();
    AngularKalmanData angular_kalman = data.angular_kalman_data.getRecentUnsync();
    CameraData cam_data = data.cam_data.getRecentUnsync();

    packet.lat = gps.latitude;
    packet.lon = gps.longitude;

    packet.alt = (((int16_t) gps.altitude) & 0xfffe) | (received_count & 0x0001);    // Convert range of value so that we can also account for negative altitudes (converting float to 15 bit alt, 1 bit command ack)
    
    
    packet.baro_alt = inv_convert_range<int16_t>(barometer.altitude, 1 << 17);
  
    //auto [ax,ay,az, tilt_extra] = pack_highg_tilt(highg, map(static_cast<long>(orientation.tilt * 100),0, 314, 0, 1023));
    packet.highg_ax = (uint16_t)inv_convert_range<int16_t>(imu.highg_acceleration.ax, MAX_ABS_ACCEL_RANGE_G);
    packet.highg_ay = (uint16_t)inv_convert_range<int16_t>(imu.highg_acceleration.ay, MAX_ABS_ACCEL_RANGE_G);
    packet.highg_az = (uint16_t)inv_convert_range<int16_t>(imu.highg_acceleration.az, MAX_ABS_ACCEL_RANGE_G);


    //-------------------------------------------------------------------

    // Tilt & FSM State --> comp_tilt vs mq_tilt
    static_assert(FSMState::FSM_STATE_COUNT < 16);
    uint16_t tilt_norm = (angular_kalman.comp_tilt / M_PI) * 0x0fff; // Encodes tilt value 0-1 into range 0x0000 - 0x0fff
    packet.tilt_fsm |= ((tilt_norm << 4) & 0xfff0);
    packet.tilt_fsm |= ((uint16_t)fsm & 0x000f);

    //-------------------------------------------------------------------

    //Serial.println(packet.tilt_fsm, 2);
    // Battery voltage
    packet.batt_volt = inv_convert_range<uint8_t>(voltage.v_Bat, MAX_TELEM_VOLTAGE_V);
    
    // Roll rate
    float roll_rate_hz = std::clamp(std::abs(imu.angular_velocity.vx) / 360.0f, 0.0f, MAX_ROLL_RATE_HZ); // divide by 360 to convert from dps to Hz
    packet.roll_rate = roll_rate_hz / MAX_ROLL_RATE_HZ * 0xFF;

    // KF data
    packet.kf_px = inv_convert_range<int16_t>(kalman.position.px, MAX_KF_VPOSITION_M); // We should eventually switch this to unsigned, once we fix baro.
    packet.kf_py = inv_convert_range<int16_t>(kalman.position.py, MAX_KF_LPOSITION_M);
    packet.kf_pz = inv_convert_range<int16_t>(kalman.position.pz, MAX_KF_LPOSITION_M);

    packet.kf_vx = inv_convert_range<int16_t>(kalman.velocity.vx, MAX_KF_XVELOCITY_MS);

    //Camera Data
    packet.camera_state = cam_data.camera_state;
    // bit shift by 4 to give max resolution while still keeping possible voltage up to >=12V
    packet.camera_batt_volt = (float)((cam_data.camera_voltage) / 9) * 0xFF;

    
    //Pyro A0 | B1 | C2 | D3
    packet.pyro |= (uint32_t)((std::clamp(voltage.continuity[0], 0.0f, MAX_TELEM_VOLTAGE_V) / MAX_TELEM_VOLTAGE_V) * 255) << (0*8);
    packet.pyro |= (uint32_t)((std::clamp(voltage.continuity[1], 0.0f, MAX_TELEM_VOLTAGE_V) / MAX_TELEM_VOLTAGE_V) * 255) << (1*8);
    packet.pyro |= (uint32_t)((std::clamp(voltage.continuity[2], 0.0f, MAX_TELEM_VOLTAGE_V) / MAX_TELEM_VOLTAGE_V) * 255) << (2*8);
    packet.pyro |= (uint32_t)((std::clamp(voltage.continuity[3], 0.0f, MAX_TELEM_VOLTAGE_V) / MAX_TELEM_VOLTAGE_V) * 255) << (3*8);

    // GPS state & Callsign
    // 0000 | 000 | 0
    // SATC | FT  | C
    packet.callsign_gpsfix_satcount |= (gps.fix_type & 0x07) << 1;
    packet.callsign_gpsfix_satcount |= (gps.sats_in_view & 0x0F) << 4;

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