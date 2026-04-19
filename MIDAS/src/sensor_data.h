#pragma once

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>

#include "finite-state-machines/fsm_config.h"
#include "data_logging_meta.h"

/**
 * @brief
 * This header provides all the implementation for the data that comes from all of the sensors/
 * These structs will be individual packets of data passed between the sensor and the 
 * rocket_state struct, and each will be tagged with a timestamp.
*/

/**
 * @brief First 4 structs are base vector, pos, vel, and accel data to be used elsewhere
*/
struct Vec3 {
    float x = 0;
    float y = 0;
    float z = 0;
};

struct Position {
    float px = 0;
    float py = 0;
    float pz = 0;
};

struct Velocity {
    float vx = 0;
    float vy = 0;
    float vz = 0;

    float get_speed() {
        return sqrt(vx * vx + vy * vy + vz * vz);
    }
};

struct Acceleration {
    float ax = 0;
    float ay = 0;
    float az = 0;

    // Get G-Force applied on the rocket
    float get_magnitude() {
        return sqrt(ax * ax + ay * ay + az * az);
    }
};

/**
 * @struct euler_t
 * 
 * @brief euler representation of rotation
*/
struct euler_t {
    float yaw;
    float pitch;
    float roll;
};

/**
 * @brief Structs starting here represent specific sensors and the respective data
*/

/**
 * @struct Barometer
 * 
 * @brief data from the barometer
*/
struct Barometer {
    float temperature = 0; // Temperature in Celcius
    uint32_t pressure = 0; // Pressure in Pascals
    float altitude = 0; // Altitude in meters (above sea level?)

    Barometer() = default;
    Barometer(float t, float p, float a) : temperature(t), pressure(p), altitude(a) {}
};


/**
 * @struct Voltage
 * 
 * @brief data about battery voltage
*/
struct Voltage {
    float continuity[4];
    float v_Bat = 0;
    float v_Pyro = 0;
};

/**
 * @struct GPS
 * 
 * @brief data from the GPS
*/
struct GPS {
    int32_t latitude = 0;
    int32_t longitude = 0;
    float altitude = 0; // Altitude in meters
    float speed = 0; // Speed in meters/second
    uint16_t fix_type = 0;
    uint16_t sats_in_view = 0;
    // Unix timestamp since 1970
    // This isn't included in the telem packet because this is
    // solely for the SD logger. We do not need to know what time it is
    // when we are recieving telem packets.
    uint32_t time;
};

/**
 * @struct Magnetometer
 * 
 * @brief data from the magnetometer
*/
struct Magnetometer {
    double mx;
    double my;
    double mz;
};

struct Quaternion { //long term, remove or rename this struct, it will conflict with libraries where Quaternion is well defined. 
    float w, x, y, z;

    static float dot(const Quaternion& q1, const Quaternion& q2) {
        return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
    }

};

/**
 * @enum Orientation reading type
 */
enum class OrientationReadingType {
    FULL_READING = 0,
    ANGULAR_VELOCITY_UPDATE = 1
};

/**
 * @struct SFLP
 * 
 * @brief Data from the LSM6DSV320X Sensor Fusion Low Power module
 * 
 */
struct IMU_SFLP {
    Quaternion quaternion;
    Acceleration gravity;
    Velocity gyro_bias;
};


enum class MetalogSummaryEntryType{
    CURRENT,
    MAXIMUM,
    MINIMUM,
};

template<typename T>
class MetalogSummaryEntry {

    public:

    MetalogSummaryEntry(const MetaDataCode &code, const MetalogSummaryEntryType &type = MetalogSummaryEntryType::CURRENT, const T &default_val = T()){
        code = code;
        type = type;
        data = default_val;
    }

    void update(const T &newval){
        switch(type){
            case MetalogSummaryEntryType::CURRENT:
                data = newval;
                break;
            case MetalogSummaryEntryType::MAXIMUM:
                if (newval > data){
                    data = newval;
                }
                break;
            case MetalogSummaryEntryType::MINIMUM:
                if (newval < data){
                    data = newval;
                }
                break;
        }
    }

    void commit(MetaLogging &metalog){
        metalog.log_data(code, data);
    }

    private:
        T data;
        MetaDataCode code;
        MetalogSummaryEntryType type;  
};

struct MetalogSummary{
    // Launch events
    MetalogSummaryEntry<uint32_t> event_tlaunch {MetaDataCode::EVENT_TLAUNCH};
    MetalogSummaryEntry<uint32_t> event_tburnout {MetaDataCode::EVENT_TBURNOUT};
    MetalogSummaryEntry<uint32_t> event_tignition {MetaDataCode::EVENT_TIGNITION};
    MetalogSummaryEntry<uint32_t> event_tapogee {MetaDataCode::EVENT_TAPOGEE};
    MetalogSummaryEntry<uint32_t> event_tmain {MetaDataCode::EVENT_TMAIN};
    MetalogSummaryEntry<uint32_t> event_tmax_accel {MetaDataCode::EVENT_TMAX_ACCEL};
    MetalogSummaryEntry<uint32_t> event_tmax_vel {MetaDataCode::EVENT_TMAX_VEL};
    MetalogSummaryEntry<uint32_t> event_tmax_descent_rate {MetaDataCode::EVENT_TMAX_DESCENT_RATE};

    // Non-events
    MetalogSummaryEntry<float> data_launchsite_baro {MetaDataCode::DATA_LAUNCHSITE_BARO};
    MetalogSummaryEntry<float> data_launchsite_gps {MetaDataCode::DATA_LAUNCHSITE_GPS};
    MetalogSummaryEntry<float> data_launch_initial_tilt {MetaDataCode::DATA_LAUNCH_INITIAL_TILT};
    MetalogSummaryEntry<float> data_tilt_at_burnout {MetaDataCode::DATA_TILT_AT_BURNOUT};
    MetalogSummaryEntry<float> data_tilt_at_ignition {MetaDataCode::DATA_TILT_AT_IGNITION};
    MetalogSummaryEntry<float> data_max_accel {MetaDataCode::DATA_MAX_ACCEL, MetalogSummaryEntryType::MAXIMUM, -std::numeric_limits<float>::max()};
    MetalogSummaryEntry<float> data_max_vel {MetaDataCode::DATA_MAX_VEL, MetalogSummaryEntryType::MAXIMUM, -std::numeric_limits<float>::max()};
    MetalogSummaryEntry<float> data_alt_at_burnout {MetaDataCode::DATA_ALT_AT_BURNOUT};
    MetalogSummaryEntry<float> data_max_descent_rate {MetaDataCode::DATA_MAX_DESCENT_RATE, MetalogSummaryEntryType::MAXIMUM, -std::numeric_limits<float>::max()};
};

/**
 * 
 * @struct IMU
 * 
 * @brief IMU that stores High/Low G Acceleration, Angular Velocity, and IMU_SFLP
 * 
 */
struct IMU { 
    Acceleration highg_acceleration;
    Acceleration lowg_acceleration;
    Velocity angular_velocity;
};

// adding this to dissect how to get FIFO
/*
    lsm6dsv320x_fifo_out_raw_t f_data;      __initial variable

      lsm6dsv320x_fifo_out_raw_get(&dev_ctx, &f_data);    __function for getting the data || put this in fifo_out_raw_get  
      datax = (int16_t *)&f_data.data[0];
      datay = (int16_t *)&f_data.data[2];
      dataz = (int16_t *)&f_data.data[4];      
      
      ts = (int32_t *)&f_data.data[0];      


    All of the sums are for an average

    __GBIAS

    axis = &f_data.data[0];
    gbias_mdps[0] = lsm6dsv320x_from_fs125_to_mdps(axis[0] | (axis[1] << 8));   __getting the gbias (x, y, z)
    gbias_mdps[1] = lsm6dsv320x_from_fs125_to_mdps(axis[2] | (axis[3] << 8));
    gbias_mdps[2] = lsm6dsv320x_from_fs125_to_mdps(axis[4] | (axis[5] << 8));

    gbias_sum[0] += gbias_mdps[0];
    gbias_sum[1] += gbias_mdps[1];
    gbias_sum[2] += gbias_mdps[2];
    
    remove {
    gbias_cnt++;    __this gets the count to eventually divide the average, we do not need these
    break;
    }


    __GRAVITY VECTOR

    axis = &f_data.data[0];
    gravity_mg[0] = lsm6dsv320x_from_sflp_to_mg(axis[0] | (axis[1] << 8));
    gravity_mg[1] = lsm6dsv320x_from_sflp_to_mg(axis[2] | (axis[3] << 8));
    gravity_mg[2] = lsm6dsv320x_from_sflp_to_mg(axis[4] | (axis[5] << 8));

    gravity_sum[0] += gravity_mg[0];
    gravity_sum[1] += gravity_mg[1];
    gravity_sum[2] += gravity_mg[2];
    
    // remove {
    gravity_cnt++;
    break;


    __QUATERNION

        uint16_t *sflp = (uint16_t *)&f_data.data[2];

        if (f_data.data[0] == 0x00) {

            __game rotation first word
          quat[0] = npy_half_to_float(sflp[0]);
          quat[1] = npy_half_to_float(sflp[1]);
        } else if (f_data.data[0] == 0x01) {
            
        __game rotation second word 
          quat[2] = npy_half_to_float(sflp[0]);
          quat[3] = npy_half_to_float(sflp[1]);

          rot_sum[0] += quat[0];
          rot_sum[1] += quat[1];
          rot_sum[2] += quat[2];
          rot_sum[3] += quat[3];

          rot_cnt++;
        } else {

            __error
          snprintf((char *)tx_buffer, sizeof(tx_buffer), "[%02x - %02x] wrong word \r\n", f_data.data[0], f_data.data[1]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
       }

        break;
      }
    
}
*/

/**
 * @struct KalmanData
 * 
 * @brief data from the Kalman thread
*/
struct KalmanData {
    Position position;
    Velocity velocity;
    Acceleration acceleration;
};

/**
 * @struct KalmanData
 * 
 * @brief data from the MQEKF thread
*/
struct AngularKalmanData {
    Quaternion quaternion;
    float gyrobias[3];
    double sflp_tilt = 0.0;
    double mq_tilt = 0.0;
    bool has_data = false;
    
    float yaw = 0;
    float pitch = 0;
    float roll = 0;
    // For yessir.cpp
    euler_t getEuler() const {
        euler_t euler;
        euler.yaw = this->yaw;
        euler.pitch = this->pitch;
        euler.roll = this->roll;
        return euler;
    }
    
};

/**
 * @struct PyroState
 * 
 * @brief data regarding all pyro channels
*/
struct PyroState {
    bool is_global_armed = false;
    bool channel_firing[MIDAS_NUM_PYROS];      // Whether the pyro is currently firing
    bool pyro_event_consumed[MIDAS_NUM_PYROS]; // Whether the pyro has attempted to fire in flight already.

    /**
     * By convention, the pyro states are as follows:
     * [0] PYRO A / APOGEE
     * [1] PYRO B / MAIN
     * [2] PYRO C / MOTOR
     * [3] PYRO D / AUX
     */
};


struct CameraData {
    uint8_t camera_state = 255;
    float camera_voltage = 0;
};
