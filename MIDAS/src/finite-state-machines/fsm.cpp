#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>

#include <ArduinoJson.h>
#include "fsm.h"



static double sustainer_idle_to_first_boost_acceleration_threshold;
static double sustainer_idle_to_first_boost_time_threshold;
static double sustainer_ignition_to_second_boost_acceleration_threshold;
static double sustainer_ignition_to_second_boost_time_threshold;
static double sustainer_second_boost_to_coast_time_threshold;
static double sustainer_coast_detection_acceleration_threshold;
static double sustainer_coast_to_apogee_vertical_speed_threshold;
static double sustainer_apogee_backto_coast_vertical_speed_threshold;
static double sustainer_apogee_check_threshold;
static double sustainer_apogee_timer_threshold;
static double sustainer_drogue_timer_threshold;
static double sustainer_main_to_main_deploy_timer_threshold;
static double sustainer_main_deploy_altitude_threshold;
static double sustainer_ignition_to_coast_timer_threshold;
static double sustainer_landed_timer_threshold;
static double sustainer_landed_vertical_speed_threshold;
static double sustainer_first_boost_to_burnout_time_threshold;
static double sustainer_drogue_jerk_threshold;
static double sustainer_main_jerk_threshold;

static double booster_idle_to_first_boost_acceleration_threshold;
static double booster_idle_to_first_boost_time_threshold;
static double booster_first_seperation_time_threshold;
static double booster_coast_detection_acceleration_threshold;
static double booster_coast_to_apogee_vertical_speed_threshold;
static double booster_apogee_check_threshold;
static double booster_apogee_timer_threshold;
static double booster_drogue_timer_threshold;
static double booster_main_to_main_deploy_timer_threshold;
static double booster_main_deploy_altitude_threshold;
static double booster_ignition_to_second_boost_time_threshold;
static double booster_ignition_to_coast_timer_threshold;
static double booster_landed_timer_threshold;
static double booster_first_boost_to_burnout_time_threshold;
static double booster_landed_vertical_speed_threshold;
static double booster_first_separation_jerk_threshold;
static double booster_drogue_jerk_threshold;
static double booster_main_jerk_threshold;

#include <SPIFFS.h> 

static bool jsonLoaded = false;

static void loadJsonThresholds() {
    if (jsonLoaded) {
        return;  
    }

    if (!SPIFFS.begin(true)) { 
        Serial.println("⚠️ SPIFFS mount failed! Ensure SPIFFS is enabled.");
        return;
    }

    File file = SPIFFS.open("/sg1-4.json", "r");
    if (!file) {
        Serial.println("❌ Error: Could not open /sg1-4.json");
        return;
    }

    // Create a JSON document (adjust size as needed)
    StaticJsonDocument<2048> doc;
    
    // Deserialize JSON
    DeserializationError error = deserializeJson(doc, file);
    file.close();  // Close file after parsing

    if (error) {
        Serial.print("❌ JSON parsing failed: ");
        Serial.println(error.f_str());
        return;
    }

    // Load second stage thresholds
    JsonObject s = doc["second_stage_thresholds"];
    sustainer_idle_to_first_boost_acceleration_threshold      = s["idle_to_first_boost"]["acceleration_threshold"] | 3;
    sustainer_idle_to_first_boost_time_threshold             = s["idle_to_first_boost"]["time_threshold"] | 1000;
    sustainer_ignition_to_second_boost_acceleration_threshold = s["ignition_to_second_boost"]["acceleration_threshold"] | 3;
    sustainer_ignition_to_second_boost_time_threshold         = s["ignition_to_second_boost"]["time_threshold"] | 1000;
    sustainer_second_boost_to_coast_time_threshold            = s["second_boost_to_coast"]["time_threshold"] | 1000;
    sustainer_coast_detection_acceleration_threshold          = s["coast_detection"]["acceleration_threshold"] | 0.2;
    sustainer_coast_to_apogee_vertical_speed_threshold        = s["coast_to_apogee"]["vertical_speed_threshold"] | 15;
    sustainer_apogee_backto_coast_vertical_speed_threshold    = s["apogee"]["backto_coast_vertical_speed_threshold"] | 10;
    sustainer_apogee_check_threshold                          = s["apogee"]["check_threshold"] | 1000;
    sustainer_apogee_timer_threshold                          = s["apogee"]["timer_threshold"] | 1000;
    sustainer_drogue_timer_threshold                          = s["drogue"]["timer_threshold"] | 3000;
    sustainer_main_to_main_deploy_timer_threshold             = s["main"]["to_main_deploy_timer_threshold"] | 3000;
    sustainer_main_deploy_altitude_threshold                  = s["main"]["deploy_altitude_threshold"] | 3000;
    sustainer_ignition_to_coast_timer_threshold               = s["ignition"]["to_coast_timer_threshold"] | 5000;
    sustainer_landed_timer_threshold                          = s["landed"]["timer_threshold"] | 5000;
    sustainer_landed_vertical_speed_threshold                 = s["landed"]["vertical_speed_threshold"] | 20;
    sustainer_first_boost_to_burnout_time_threshold           = s["first_boost"]["to_burnout_time_threshold"] | 1000;
    sustainer_drogue_jerk_threshold                           = s["drogue_jerk_threshold"] | 200;
    sustainer_main_jerk_threshold                             = s["main_jerk_threshold"] | 300;

    // Load first stage thresholds
    JsonObject b = doc["first_stage_thresholds"];
    booster_idle_to_first_boost_acceleration_threshold    = b["idle_to_first_boost"]["acceleration_threshold"] | 3;
    booster_idle_to_first_boost_time_threshold            = b["idle_to_first_boost"]["time_threshold"] | 1000;
    booster_first_seperation_time_threshold               = b["first_separation"]["time_threshold"] | 3000;
    booster_coast_detection_acceleration_threshold        = b["coast_detection"]["acceleration_threshold"] | 0.2;
    booster_coast_to_apogee_vertical_speed_threshold       = b["coast_to_apogee"]["vertical_speed_threshold"] | 20;
    booster_apogee_check_threshold                         = b["apogee"]["check_threshold"] | 1000;
    booster_apogee_timer_threshold                         = b["apogee"]["timer_threshold"] | 1000;
    booster_drogue_timer_threshold                         = b["drogue"]["timer_threshold"] | 3000;
    booster_main_to_main_deploy_timer_threshold            = b["main"]["to_main_deploy_timer_threshold"] | 3000;
    booster_main_deploy_altitude_threshold                 = b["main"]["deploy_altitude_threshold"] | 3000;
    booster_ignition_to_second_boost_time_threshold        = b["ignition"]["to_second_boost_time_threshold"] | 1000;
    booster_ignition_to_coast_timer_threshold              = b["ignition"]["to_coast_timer_threshold"] | 5000;
    booster_landed_timer_threshold                         = b["landed"]["timer_threshold"] | 5000;
    booster_first_boost_to_burnout_time_threshold          = b["first_boost"]["to_burnout_time_threshold"] | 1000;
    booster_landed_vertical_speed_threshold                = b["landed"]["vertical_speed_threshold"] | 20;
    booster_first_separation_jerk_threshold                = b["first_separation_jerk_threshold"] | 300;
    booster_drogue_jerk_threshold                          = b["drogue_jerk_threshold"] | 200;
    booster_main_jerk_threshold                            = b["main_jerk_threshold"] | 300;

    jsonLoaded = true;
    Serial.println("✅ JSON Thresholds Loaded Successfully!");
}



/**
 * @brief Helper to calculate the average value of a buffered sensor data
 * 
 * @param sensor Buffered sensor struct
 * @param get_item Lambda get function 
 * 
 * @return Average value
*/

template<typename T, size_t count>
double sensor_average(BufferedSensorData<T, count>& sensor, double (* get_item)(T&)) {
    auto arr = sensor.template getBufferRecent<count>();
    double sum = 0.0;
    for (T& item : arr) {
        sum += get_item(item);
    }
    return sum / count;
}

/**
 * @brief Helper to calculate the derivative over a buffered sensor data
 * 
 * @param sensor Buffered sensor struct
 * @param get_item Lambda get function 
 * 
 * @return Derivative
*/
template<typename T, size_t count>
double sensor_derivative(BufferedSensorData<T, count>& sensor, double (* get_item)(T&)) {
    auto arr   = sensor.template getBufferRecent<count>();
    auto times = sensor.template getTimesRecent<count>();
    size_t i   = 0;

    double first_average      = 0.0;
    double first_average_time = 0.0;
    for (; i < count / 2; i++) {
        first_average      += get_item(arr[i]);
        first_average_time += pdTICKS_TO_MS(times[i]) / 1000.0;
    }
    first_average      /= (count / 2.0);
    first_average_time /= (count / 2.0);

    double second_average      = 0.0;
    double second_average_time = 0.0;
    for (; i < count; i++) {
        second_average      += get_item(arr[i]);
        second_average_time += pdTICKS_TO_MS(times[i]) / 1000.0;
    }
    second_average      /= (count / 2.0);
    second_average_time /= (count / 2.0);

    return (second_average - first_average) / (second_average_time - first_average_time);
}

/**
 * @brief Populates StateEstimate struct with the correct values for accel, alt, jerk, and speed
*/
StateEstimate::StateEstimate(RocketData& state) {
    acceleration = sensor_average<HighGData, 8>(state.high_g, [](HighGData& data) {
        return (double) data.ax;
    });
    altitude = sensor_average<Barometer, 8>(state.barometer, [](Barometer& data) {
        return (double) data.altitude;
    });
    jerk = sensor_derivative<HighGData, 8>(state.high_g, [](HighGData& data) {
        return (double) data.ax;
    });
    vertical_speed = sensor_derivative<Barometer, 8>(state.barometer, [](Barometer& data) {
        return (double) data.altitude;
    });
}


#ifdef IS_SUSTAINER



/**
 * @brief Sustainer FSM tick function, which will advance the current state if necessary
 * 
 * @param state current FSM state
 * @param state_estimate StateEstimate struct for the current estimate for accel, alt, jerk, and speed
 * 
 * @return New FSM State
*/

FSMState FSM::tick_fsm(FSMState& state, StateEstimate state_estimate) {
    // 1) Ensure thresholds are loaded
    loadJsonThresholds();

    // 2) get current time
    double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

    switch (state) {
        case FSMState::STATE_IDLE:
            if (state_estimate.acceleration > sustainer_idle_to_first_boost_acceleration_threshold) {
                launch_time = current_time;
                state       = FSMState::STATE_FIRST_BOOST;
            }
            break;

        case FSMState::STATE_FIRST_BOOST:
            if ((state_estimate.acceleration < sustainer_idle_to_first_boost_acceleration_threshold) &&
                ((current_time - launch_time) < sustainer_idle_to_first_boost_time_threshold)) {
                state = FSMState::STATE_IDLE;
                break;
            }
            if (state_estimate.acceleration < sustainer_coast_detection_acceleration_threshold) {
                burnout_time = current_time;
                state        = FSMState::STATE_BURNOUT;
            }
            break;

        case FSMState::STATE_BURNOUT:
            if ((state_estimate.acceleration >= sustainer_coast_detection_acceleration_threshold) &&
                ((current_time - burnout_time) < sustainer_first_boost_to_burnout_time_threshold)) {
                state = FSMState::STATE_FIRST_BOOST;
                break;
            }
            if ((current_time - burnout_time) > sustainer_first_boost_to_burnout_time_threshold) {
                sustainer_ignition_time = current_time;
                state = FSMState::STATE_SUSTAINER_IGNITION;
            }
            break;

        case FSMState::STATE_SUSTAINER_IGNITION:
            if ((current_time - sustainer_ignition_time) > sustainer_ignition_to_coast_timer_threshold) {
                coast_time = current_time;
                state      = FSMState::STATE_COAST;
                break;
            }
            if (state_estimate.acceleration > sustainer_ignition_to_second_boost_acceleration_threshold) {
                second_boost_time = current_time;
                state             = FSMState::STATE_SECOND_BOOST;
            }
            break;

        case FSMState::STATE_SECOND_BOOST:
            if ((state_estimate.acceleration < sustainer_ignition_to_second_boost_acceleration_threshold) &&
                ((current_time - second_boost_time) < sustainer_ignition_to_second_boost_time_threshold)) {
                state = FSMState::STATE_SUSTAINER_IGNITION;
                break;
            }
            if (state_estimate.acceleration < sustainer_coast_detection_acceleration_threshold) {
                coast_time = current_time;
                state      = FSMState::STATE_COAST;
            }
            break;

        case FSMState::STATE_COAST:
            if ((state_estimate.acceleration > sustainer_coast_detection_acceleration_threshold) &&
                ((current_time - coast_time) < sustainer_second_boost_to_coast_time_threshold)) {
                state = FSMState::STATE_SECOND_BOOST;
                break;
            }
            if (state_estimate.vertical_speed <= sustainer_coast_to_apogee_vertical_speed_threshold) {
                apogee_time = current_time;
                state       = FSMState::STATE_APOGEE;
            }
            break;

        case FSMState::STATE_APOGEE:
            if ((state_estimate.vertical_speed) > sustainer_apogee_backto_coast_vertical_speed_threshold &&
                ((current_time - apogee_time) < sustainer_apogee_check_threshold)) {
                state = FSMState::STATE_COAST;
                break;
            }
            if ((current_time - apogee_time) > sustainer_apogee_timer_threshold) {
                drogue_time = current_time;
                state       = FSMState::STATE_DROGUE_DEPLOY;
            }
            break;

        case FSMState::STATE_DROGUE_DEPLOY:
            if (std::abs(state_estimate.jerk) < sustainer_drogue_jerk_threshold) {
                state = FSMState::STATE_DROGUE;
                break;
            }
            if ((current_time - drogue_time) > sustainer_drogue_timer_threshold) {
                state = FSMState::STATE_DROGUE;
            }
            break;

        case FSMState::STATE_DROGUE:
            if (state_estimate.altitude <= sustainer_main_deploy_altitude_threshold) {
                state     = FSMState::STATE_MAIN_DEPLOY;
                main_time = current_time;
            }
            break;

        case FSMState::STATE_MAIN_DEPLOY:
            if (std::abs(state_estimate.jerk) < sustainer_main_jerk_threshold) {
                state = FSMState::STATE_MAIN;
                break;
            }
            if ((current_time - main_time) > sustainer_main_to_main_deploy_timer_threshold) {
                state = FSMState::STATE_MAIN;
            }
            break;

        case FSMState::STATE_MAIN:
            if (std::abs(state_estimate.vertical_speed) <= sustainer_landed_vertical_speed_threshold) {
                landed_time = current_time;
                state       = FSMState::STATE_LANDED;
            }
            break;

        case FSMState::STATE_LANDED:
            if ((std::abs(state_estimate.vertical_speed) > sustainer_landed_vertical_speed_threshold) &&
                ((current_time - landed_time) > sustainer_landed_timer_threshold)) {
                state = FSMState::STATE_MAIN;
            }
            break;
    }
    return state;
}
#else


/**
 * @brief Booster FSM tick function, which will advance the current state if necessary
 * 
 * This is similar to the previous function but contains less states
 * 
 * @param state current FSM state
 * @param state_estimate StateEstimate struct for the current estimate for accel, alt, jerk, and speed
 * 
 * @return New FSM State
*/

FSMState FSM::tick_fsm(FSMState& state, StateEstimate state_estimate) {
    // 1) Ensure thresholds are loaded
    loadJsonThresholds();

    double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

    switch (state) {
        case FSMState::STATE_IDLE:
            if (state_estimate.acceleration > booster_idle_to_first_boost_acceleration_threshold) {
                launch_time = current_time;
                state       = FSMState::STATE_FIRST_BOOST;
            }
            break;

        case FSMState::STATE_FIRST_BOOST:
            if ((state_estimate.acceleration < booster_idle_to_first_boost_acceleration_threshold) &&
                ((current_time - launch_time) < booster_idle_to_first_boost_time_threshold)) {
                state = FSMState::STATE_IDLE;
                break;
            }
            if (state_estimate.acceleration < booster_coast_detection_acceleration_threshold) {
                burnout_time = current_time;
                state        = FSMState::STATE_BURNOUT;
            }
            break;

        case FSMState::STATE_BURNOUT:
            if ((state_estimate.acceleration >= booster_coast_detection_acceleration_threshold) &&
                ((current_time - burnout_time) < booster_first_boost_to_burnout_time_threshold)) {
                state = FSMState::STATE_FIRST_BOOST;
                break;
            }
            if ((current_time - burnout_time) > booster_first_boost_to_burnout_time_threshold) {
                first_separation_time = current_time;
                state                 = FSMState::STATE_FIRST_SEPARATION;
            }
            break;

        case FSMState::STATE_FIRST_SEPARATION:
            if (std::abs(state_estimate.jerk) < booster_first_separation_jerk_threshold) {
                state = FSMState::STATE_COAST;
                break;
            }
            if ((current_time - first_separation_time) > booster_first_seperation_time_threshold) {
                state = FSMState::STATE_COAST;
            }
            break;

        case FSMState::STATE_COAST:
            if (state_estimate.vertical_speed <= booster_coast_to_apogee_vertical_speed_threshold) {
                apogee_time = current_time;
                state       = FSMState::STATE_APOGEE;
            }
            break;

        case FSMState::STATE_APOGEE:
            if ((state_estimate.vertical_speed > booster_coast_to_apogee_vertical_speed_threshold) &&
                ((current_time - apogee_time) < booster_apogee_check_threshold)) {
                state = FSMState::STATE_COAST;
                break;
            }
            if ((current_time - apogee_time) > booster_apogee_timer_threshold) {
                drogue_time = current_time;
                state       = FSMState::STATE_DROGUE_DEPLOY;
            }
            break;

        case FSMState::STATE_DROGUE_DEPLOY:
            if (std::abs(state_estimate.jerk) < booster_drogue_jerk_threshold) {
                state = FSMState::STATE_DROGUE;
                break;
            }
            if ((current_time - drogue_time) > booster_drogue_timer_threshold) {
                state = FSMState::STATE_DROGUE;
            }
            break;

        case FSMState::STATE_DROGUE:
            if (state_estimate.altitude <= booster_main_deploy_altitude_threshold) {
                state     = FSMState::STATE_MAIN_DEPLOY;
                main_time = current_time;
            }
            break;

        case FSMState::STATE_MAIN_DEPLOY:
            if (std::abs(state_estimate.jerk) < booster_main_jerk_threshold) {
                state = FSMState::STATE_MAIN;
                break;
            }
            if ((current_time - main_time) > booster_main_to_main_deploy_timer_threshold) {
                state = FSMState::STATE_MAIN;
            }
            break;

        case FSMState::STATE_MAIN:
            if (std::abs(state_estimate.vertical_speed) <= booster_landed_vertical_speed_threshold) {
                landed_time = current_time;
                state       = FSMState::STATE_LANDED;
            }
            break;

        case FSMState::STATE_LANDED:
            if ((std::abs(state_estimate.vertical_speed) > booster_landed_vertical_speed_threshold) &&
                ((current_time - landed_time) > booster_landed_timer_threshold)) {
                state = FSMState::STATE_MAIN;
            }
            break;
    }
    return state;
}
#endif
