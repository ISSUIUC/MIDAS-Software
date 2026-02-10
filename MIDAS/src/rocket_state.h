#pragma once

#include <array>

#include "sensor_data.h"
#include "hal.h"
#include "Buffer.h"

/** 
 * @brief The RocketState struct stores everything that is needed by more than one system/thread of the Rocket.
 *
 * @note
 *  Normally, this would be considered a poor decision. However, the fact that all this data is here
 *  makes it easier to debug since all this data can be logged (and thus used when debugging).
 * 
 */

/**
 * @struct Reading
 * 
 * @brief A specific reading from a sensor, holding a timestamp and data
 * 
 * @tparam SensorData Type of data we are storing
*/
template<typename SensorData>
struct Reading {
    uint32_t timestamp_ms;
    SensorData data;
};

/**
 * @struct SensorData 
 * 
 * @brief Wrapper for thread safe sensor data storage
 * 
 * @tparam S Type of data we are storing
*/
template<typename S>
struct SensorData {
private:
    Mutex<S> current;
    Queue<Reading<S>> queue;

public:
    SensorData() : current(S()) { }

    /**
     * @brief pushes to the queue with new data
     * 
     * @param data New data of the templated type to add to queue
    */
    virtual void update(S data) {
        current.write(data);
        queue.send((Reading<S>) { .timestamp_ms = pdTICKS_TO_MS(xTaskGetTickCount()), .data = data });
    };

    /**
     * @brief gets most recent data, will wait and acquire lock
     * 
     * @return the most recent data
    */
    S getRecent() {
        return current.read();
    };

    /**
     * @brief gets most recent data, will not acquire lock
     * 
     * @return the most recent data
    */
    S getRecentUnsync() {
        return current.read_unsync();
    }

    /**
     * @brief destructively gets the queue
     * 
     * @note
     * This will clear the entire queue
     * 
     * @param out Array in which to put the queue data
     * 
     * @return boolean indicating successful transfer of data
    */
    bool getQueued(Reading<S>* out) {
        return queue.receive(out);
    };
};

/**
 * @struct BufferedSensorData
 * 
 * @brief Buffer of Sensor data for easy calculations on
 * 
 * @tparam S type of data stored
 * @tparam count size of buffer
*/
template<typename S, size_t count>
struct BufferedSensorData : public SensorData<S> {
private:
    Buffer<S, count> buffer;
    Buffer<TickType_t, count> data_time;

public:
    BufferedSensorData() : SensorData<S>() { }

    void update(S data) override {
        SensorData<S>::update(data);
        buffer.push(data);
        data_time.push(xTaskGetTickCount());
    };

    // wrapper function to get easy access to buffer data
    template<size_t arr_count>
    std::array<S, arr_count> getBufferRecent() {
        std::array<S, arr_count> arr = buffer. template read_recent<arr_count>();
        return arr;
    };

    template<size_t arr_count>
    std::array<TickType_t, arr_count> getTimesRecent() {
        std::array<TickType_t, arr_count> arr = data_time. template read_recent<arr_count>();
        return arr;
    };
};

/**
 * @class Latency
 * 
 * @brief class determining latency between a set time and when the data was created
*/
class Latency {
    uint32_t latency = 0;
    TickType_t last_tick = 0;

public:
    /**
     * @brief updates latency since last packet 
    */
    void tick() {
        TickType_t now = xTaskGetTickCount();
        latency = now - last_tick;
        last_tick = now;
    }

    /**
     * @brief gets the msot recent latency
     * 
     * @return most receent latency
    */
    [[nodiscard]] uint32_t getLatency() const {
        return latency;
    }
};

/**
 * @struct CommandFlags
 * 
 * @brief Stores the status of commands from telemetry as boolean flags, commands are set whenever the corresponding telemetry command comes in.
 * Works in both directions, say to toggle states based on FSM transitions
 */
struct CommandFlags {
    bool should_reset_kf = false;               // CommandType::RESET_KF
    bool should_transition_safe = false;        // CommandType::SWITCH_TO_SAFE
    bool should_transition_idle = false;        // CommandType::SWITCH_TO_IDLE
    bool should_transition_pyro_test = false;   // CommandType::SWITCH_TO_PYRO_TEST
    bool should_fire_pyro_a = false;            // CommandType::FIRE_PYRO_A
    bool should_fire_pyro_b = false;            // CommandType::FIRE_PYRO_B
    bool should_fire_pyro_c = false;            // CommandType::FIRE_PYRO_C
    bool should_fire_pyro_d = false;            // CommandType::FIRE_PYRO_D
    // FSM Transition commands
    bool FSM_should_set_cam_feed_cam1 = false;  // Triggered at launch (IDLE --> FIRST_BOOST)
    bool FSM_should_power_save = false;         // Triggered after 60 seconds in LANDED state.
    bool FSM_should_swap_camera_feed = false;   // Triggered COAST --> APOGEE
};
/**
 * @struct RocketData
 * 
 * @brief The RocketData struct stores all data that is needed by more than one system/thread of the Rocket.
 *
 * @note
 *  Normally, this would be considered a poor decision. However, the fact that all this data is here
 *  makes it easier to debug since all this data can be logged (and thus used when debugging).
 */
struct RocketData {
public:
    //SensorData<LSM> lsm;

    SensorData<KalmanData> kalman;
    SensorData<AngularKalmanData> angular_kalman_data;
    BufferedSensorData<IMU, 16> imu;
    SensorData<IMU_SFLP> hw_filtered;
    BufferedSensorData<Barometer, 16> barometer;
    SensorData<Continuity> continuity;
    SensorData<PyroState> pyro;
    SensorData<FSMState> fsm_state;
    SensorData<GPS> gps;
    SensorData<Magnetometer> magnetometer;
    //SensorData<Orientation> orientation;
    SensorData<Voltage> voltage;
    SensorData<CameraData> cam_data;
    
    CommandFlags command_flags;
    Latency log_latency;
};