#include <cmath>
#include <cstring>

#include "fsm.h"
#include "thresholds.h"
#include "CRC.h"


bool FSMPyroAction::conditions_met(FSMState fsm_state, float cur_tilt, uint8_t cur_motor, float cur_time_since_launch, float cur_vx) const {
    // Pyro must be enabled.
    if (!enable) { return false; }

    // Verify the required FSM state.
    if (fsm_state != fsm_trigger) { return false; }

    // Enforce maximum allowable tilt if configured.
    if (max_tilt != -1 && cur_tilt > max_tilt) { return false; }

    // Require the specified motor/stage to have completed.
    if (after_motor != 0 && cur_motor < after_motor) { return false; }

    // Check lower launch-time bound if enabled.
    if (launch_t_gt != -1 && cur_time_since_launch < launch_t_gt) { return false; }

    // Check upper launch-time bound if enabled.
    if (launch_t_lt != -1 && cur_time_since_launch > launch_t_lt) { return false; }

    // Check minimum horizontal velocity if enabled.
    if (vx_min != -1 && cur_vx < vx_min) { return false; }

    // Check maximum horizontal velocity if enabled.
    if (vx_max != -1 && cur_vx > vx_max) { return false; }

    // Every configured condition has passed.
    return true;
}

bool FSMPyroAction::soft_conditions_met(FSMState fsm_state) const {
    if (!enable) { return false; }
    if (fsm_state != fsm_trigger) { return false; }
    return true;
}


uint32_t FSMConfiguration::calculate_crc(const FSMConfiguration& cfg) {
    constexpr size_t FSMConfigurationCRCSize = sizeof(FSMUserThresholds) + (sizeof(FSMPyroAction) * MIDAS_NUM_PYROS) + sizeof(uint8_t);
    uint8_t crc_buf[FSMConfigurationCRCSize];

    // Serialize the threshold configuration.
    size_t ptr = 0;
    memcpy(crc_buf + ptr, &cfg.thresholds, sizeof(FSMUserThresholds)); ptr += sizeof(FSMUserThresholds);

    // Serialize every configured pyro action.
    for (int i = 0; i < MIDAS_NUM_PYROS; i++) {
        memcpy(crc_buf + ptr, &cfg.pyro_actions[i], sizeof(FSMPyroAction)); ptr += sizeof(FSMPyroAction);
    }

    // Serialize the configuration version.
    memcpy(crc_buf + ptr, &cfg.version_num, sizeof(uint8_t));

    // Calculate the CRC32 over the serialized configuration.
    return CRC::Calculate(crc_buf, FSMConfigurationCRCSize, CRC::CRC_32());
}

/**
 * @brief Ensure the configuration remains small enough for storage.
 *
 * This compile-time assertion prevents the configuration structure from
 * exceeding the maximum supported size.
 */
static_assert(sizeof(FSMConfiguration) <= 256);

#ifdef FSM_SIMULATOR
#include "flight-systems/sensor_data.h"
#include "util/command_flags.h"
#else
#include "flight-systems/rocket_state.h"

/**
 * @brief Computes the arithmetic mean of the most recent sensor samples.
 *
 * Retrieves the most recent @p count entries from a buffered sensor and
 * averages the value returned by the supplied accessor function.
 *
 * @tparam T Sensor data type.
 * @tparam count Number of recent samples to average.
 *
 * @param sensor Buffered sensor data source.
 * @param get_item Function that extracts the desired value from each sample.
 *
 * @return Average value over the selected samples.
 */
template<typename T, size_t count>
double sensor_average(BufferedSensorData<T, count>& sensor, double (* get_item)(T&)) {
    auto arr = sensor.template getBufferRecent<count>();
    double sum = 0.0;

    // Accumulate the selected sensor quantity.
    for (T& item : arr) {
        sum += get_item(item);
    }

    // Return the arithmetic mean.
    return sum / count;
}

/**
 * @brief Estimates the first derivative of buffered sensor data.
 *
 * The derivative is computed by averaging the first half and second half
 * of the buffered samples independently, then dividing the difference in
 * their average values by the difference in their average timestamps.
 *
 * This approach reduces the effect of measurement noise compared to using
 * only two individual samples.
 *
 * @tparam T Sensor data type.
 * @tparam count Number of samples used in the estimate.
 *
 * @param sensor Buffered sensor data source.
 * @param get_item Function that extracts the desired value from each sample.
 *
 * @return Estimated derivative.
 */
template<typename T, size_t count>
double sensor_derivative(BufferedSensorData<T, count>& sensor, double (* get_item)(T&)) {
    auto arr = sensor.template getBufferRecent<count>();
    auto times = sensor.template getTimesRecent<count>();
    size_t i = 0;

    double first_average = 0.0;
    double first_average_time = 0.0;

    // Average the first half of the samples.
    for (; i < count / 2; i++) {
        first_average += get_item(arr[i]);
        first_average_time += pdTICKS_TO_MS(times[i]) / 1000.0;
    }

    first_average /= (count / 2.0);
    first_average_time /= (count / 2.0);

    double second_average = 0.0;
    double second_average_time = 0.0;

    // Average the second half of the samples.
    for (; i < count; i++) {
        second_average += get_item(arr[i]);
        second_average_time += pdTICKS_TO_MS(times[i]) / 1000.0;
    }

    second_average /= (count / 2.0);
    second_average_time /= (count / 2.0);

    // Compute the slope between the averaged sample groups.
    return (second_average - first_average) / (second_average_time - first_average_time);
}

/**
 * @brief Constructs a state estimate from buffered sensor data.
 *
 * The estimate uses moving averages and finite-difference derivatives to
 * produce filtered values for acceleration, altitude, jerk, and vertical
 * speed. These values are subsequently used by the flight state machine.
 *
 * @param state Current rocket sensor data.
 */
StateEstimate::StateEstimate(RocketData& state) {
    // Average longitudinal acceleration.
    acceleration = sensor_average<IMU, 16>(state.imu, [](IMU& data) {
        return (double) data.highg_acceleration.ax; //CHECK WITH DIVIJ/MICHAEL
    });

    // Average altitude.
    altitude = sensor_average<Barometer, 16>(state.barometer, [](Barometer& data) {
        return (double) data.altitude;
    });

    // Estimate longitudinal jerk.
    jerk = sensor_derivative<IMU, 16>(state.imu, [](IMU& data) {
        return (double) data.highg_acceleration.ax; //CHECK WITH DIVIJ/MICHAEL
    });

    // Estimate vertical velocity from altitude history.
    vertical_speed = sensor_derivative<Barometer, 16>(state.barometer, [](Barometer& data) {
        return (double) data.altitude;
    });
}
#endif

bool FSM::set_cfg(const FSMConfiguration& new_cfg) {
    // Check if the new config has a valid CRC.

    uint32_t cfg_crc = new_cfg.crc32;
    uint32_t calculated_crc = FSMConfiguration::calculate_crc(new_cfg);

    // Reject corrupted or invalid configurations.
    if(cfg_crc != calculated_crc) { return false; }

    // Store the validated configuration.
    config = new_cfg;
    return true;
}

FSMData FSM::tick_fsm(FSMTickData& fsm_data) {
    // Cache commonly used values locally for readability and efficiency.
    double current_time = fsm_data.cur_time;
    FSMData& state_data = fsm_data.cur_state;
    FSMState& state = state_data.state;
    uint8_t& current_motor = state_data.current_motor;
    CommandFlags& commands = fsm_data.commands;
    const StateEstimate& state_estimate = fsm_data.state_estimate;
    const FSMConfiguration& config = fsm_data.fsm_config;
    const KalmanData& kf_data = fsm_data.kf_data;

    switch(state) {

        /**
         * ============================================================
         * SAFE STATE
         * ============================================================
         *
         * The vehicle is fully disarmed. The only permitted transitions
         * are into PYRO_TEST mode or ARMED mode through explicit commands.
         */
        case FSMState::STATE_SAFE:

            // Deconflict if multiple commands are processed simultaneously.
            if(commands.should_transition_safe) {
                commands.should_transition_pyro_test = false;
                commands.should_transition_armed = false;
                commands.should_transition_safe = false;
                break;
            }

            // Enter pyro test mode only through an explicit command.
            if(commands.should_transition_pyro_test) {
                state = FSMState::STATE_PYRO_TEST;
                pyro_test_entry_time = current_time;
                commands.should_transition_pyro_test = false;
            }

            // Arm the flight computer and prepare for launch detection.
            if(commands.should_transition_armed) {
                state = FSMState::STATE_ARMED;
                current_motor = 0;
                commands.should_transition_armed = false;
            }

            break;

        /**
         * ============================================================
         * PYRO TEST STATE
         * ============================================================
         *
         * Manual pyro testing mode. The system automatically exits after
         * a timeout or immediately when commanded back to SAFE.
         */
        case FSMState::STATE_PYRO_TEST:

            // Immediately return to SAFE if requested.
            if(commands.should_transition_safe) {
                state = FSMState::STATE_SAFE;
                commands.should_transition_pyro_test = false;
                commands.should_transition_armed = false;
                commands.should_transition_safe = false;
                break;
            }

            // Automatically disarm after the configured timeout.
            if((current_time - pyro_test_entry_time) > fsms_pt_disarm_t) {
                commands.should_transition_pyro_test = false;
                state = FSMState::STATE_SAFE;
            }

            break;

        /**
         * ============================================================
         * ARMED STATE
         * ============================================================
         *
         * Waiting on the launch pad for sustained acceleration that
         * indicates motor ignition.
         */
        case FSMState::STATE_ARMED:

            // Allow immediate disarm if commanded.
            if(commands.should_transition_safe) {
                state = FSMState::STATE_SAFE;
                commands.should_transition_pyro_test = false;
                commands.should_transition_armed = false;
                commands.should_transition_safe = false;
                break;
            }

            // Detect launch using longitudinal acceleration.
            if (state_estimate.acceleration > fsms_boost_xl) {
                launch_time = current_time;                    // Record launch time.
                time_entered_cur_state = current_time;         // Record BOOST entry time.
                commands.FSM_should_set_cam_feed_cam1 = true;  // Switch camera feed.
                cur_state_lockin = false;                      // Begin false-detection validation.
                state = FSMState::STATE_BOOST;
            }

            break;

        /**
         * ============================================================
         * BOOST STATE
         * ============================================================
         *
         * The rocket is under motor thrust. This state validates that
         * ignition is genuine before locking in the boost phase and
         * later detects motor burnout.
         */
        case FSMState::STATE_BOOST:

            // --------------------------------------------------------
            // FALSE MOTOR IGNITION DETECTION
            // --------------------------------------------------------
            //
            // During the lock-in period, acceleration must remain above
            // the boost threshold. Otherwise, the launch detection is
            // considered false and the FSM returns to the previous state.
            //
            // --------------------------------------------------------
            if(!cur_state_lockin && (current_time - time_entered_cur_state) < fsms_boost_lockin_t) {

                if(state_estimate.acceleration < fsms_boost_xl) {
                    state = (current_motor == 0) ? FSMState::STATE_ARMED : FSMState::STATE_COAST;
                    time_entered_cur_state = current_time;
                    break;
                }

            } else {

                // Boost has remained valid long enough to be accepted.
                if(!cur_state_lockin) {
                    current_motor++;     // Count the completed ignition.
                    cur_state_lockin = true;
                }
            }

            // --------------------------------------------------------
            // MOTOR BURNOUT DETECTION
            // --------------------------------------------------------
            //
            // Once acceleration falls below the burnout threshold, begin
            // the coast phase while allowing false burnout validation.
            //
            // --------------------------------------------------------
            if (state_estimate.acceleration < fsms_burnout_xl) {
                time_entered_cur_state = current_time;
                state = FSMState::STATE_COAST;
                cur_state_lockin = false; // Prepare burnout validation.
            }

            break;

                /**
         * ============================================================
         * COAST STATE
         * ============================================================
         *
         * The rocket has completed a motor burn and is coasting. This
         * state validates burnout, detects subsequent stage ignitions for
         * multistage flights, and determines when apogee has been reached.
         */
        case FSMState::STATE_COAST: {

            // --------------------------------------------------------
            // FALSE BURNOUT DETECTION
            // --------------------------------------------------------
            //
            // Immediately after entering COAST, require acceleration to
            // remain below the burnout threshold for the configured
            // lock-in period. If thrust resumes before then, treat the
            // burnout as a false detection and return to BOOST.
            //
            if(!cur_state_lockin && (current_time - time_entered_cur_state) < fsms_burnout_lockin_t) {

                // The burnout has not yet been confirmed.
                if(state_estimate.acceleration >= fsms_burnout_xl) {
                    state = FSMState::STATE_BOOST;

                    // Continue timing from the original boost phase.
                    cur_state_lockin = true;

                    break;
                }

            } else {

                // Burnout has remained valid long enough to be accepted.
                cur_state_lockin = true;
            }

            // --------------------------------------------------------
            // NEXT STAGE IGNITION DETECTION
            // --------------------------------------------------------
            //
            // After burnout has been confirmed, monitor acceleration for
            // another ignition event. This supports multistage vehicles.
            //
            if(cur_state_lockin && state_estimate.acceleration > fsms_boost_xl) {
                time_entered_cur_state = current_time;
                cur_state_lockin = false;

                // Restart apogee detection for the next powered stage.
                apogee_detect_start = 0;

                state = FSMState::STATE_BOOST;
                break;
            }

            // --------------------------------------------------------
            // APOGEE DETECTION
            // --------------------------------------------------------
            //
            // Apogee is declared only after:
            //   1. Vertical speed falls below the configured threshold.
            //   2. Cruise lockout (if enabled) has been satisfied.
            //   3. Both conditions remain true for the required lock-in
            //      duration.
            //

            // Condition 1: Vertical speed indicates the vehicle is near apogee.
            bool apog_detect_low_speed = (state_estimate.vertical_speed <= fsms_apogee_detect_spd);

            // Condition 2: Cruise lockout requirement.
            bool apog_detect_cruise_lockout =
                (!config.thresholds.cruise_lockout_en ||
                 kf_data.velocity.vx <= fsms_cruise_lockout_spd);

            // Evaluate both apogee conditions.
            if (apog_detect_low_speed && apog_detect_cruise_lockout) {

                // Begin the consecutive detection timer.
                if (apogee_detect_start == 0) {
                    apogee_detect_start = current_time;
                }

                // Declare apogee once the conditions have remained valid
                // for the entire lock-in interval.
                if (current_time - apogee_detect_start >= fsms_apogee_lockin_t) {
                    time_entered_cur_state = current_time;
                    apogee_time = current_time;
                    state = FSMState::STATE_DROGUE;
                    apogee_detect_start = 0;

                    // Switch to the alternate camera after apogee.
                    commands.FSM_should_swap_camera_feed = true;
                }

            } else {

                // Restart the lock-in timer whenever either condition fails.
                apogee_detect_start = 0;
            }

            break;
        }

        /**
         * ============================================================
         * DROGUE STATE
         * ============================================================
         *
         * The vehicle is descending under the drogue parachute. Once the
         * configured deployment altitude is reached and the lockout period
         * has elapsed, transition to MAIN.
         */
        case FSMState::STATE_DROGUE:

            // Deploy the main parachute once the altitude threshold and
            // minimum lockout time have both been satisfied.
            if(state_estimate.altitude <= config.thresholds.main_alt &&
               (current_time - time_entered_cur_state) > fsms_main_lockout_t) {

                time_entered_cur_state = current_time;
                state = FSMState::STATE_MAIN;
            }

            break;

        /**
         * ============================================================
         * MAIN STATE
         * ============================================================
         *
         * The vehicle is descending under the main parachute. Landing is
         * detected by requiring a sufficiently low vertical speed for a
         * sustained period after the post-apogee lockout expires.
         */
        case FSMState::STATE_MAIN: {

            // Determine whether the vehicle appears stationary.
            bool landed_speed = (abs(state_estimate.vertical_speed) <= fsms_landed_detect_spd);

            // Prevent premature landing detection immediately after apogee.
            bool landed_lockout_passed = (current_time - apogee_time) > fsms_landed_t_lockout;

            if (landed_speed && landed_lockout_passed) {

                // Begin the landing confirmation timer.
                if (landed_detect_start == 0) {
                    landed_detect_start = current_time;
                }

                // Confirm landing after the vehicle has remained stationary
                // for the configured duration.
                if (current_time - landed_detect_start >= fsms_landed_entry_t) {
                    time_entered_cur_state = current_time;
                    cur_state_lockin = false;
                    state = FSMState::STATE_LANDED;
                    landed_detect_start = 0;
                }

            } else {

                // Movement detected; restart the landing timer.
                landed_detect_start = 0;
            }

            break;
        }

        /**
         * ============================================================
         * LANDED STATE
         * ============================================================
         *
         * The flight has completed. After a lock-in period the system
         * enters its post-flight behavior, including enabling power-saving
         * mode. If significant motion is detected again, the FSM returns
         * to MAIN.
         */
        case FSMState::STATE_LANDED:

            // Wait for the landing state to become fully locked in.
            if((current_time - time_entered_cur_state) > fsms_landed_t) {

                // Perform one-time landing actions.
                if(!cur_state_lockin) {
                    cur_state_lockin = true;
                    commands.FSM_should_power_save = true;
                }

                // Allow a telemetry command to safely disarm the system.
                if(commands.should_transition_safe) {
                    state = FSMState::STATE_SAFE;
                    commands.should_transition_pyro_test = false;
                    commands.should_transition_armed = false;
                    commands.should_transition_safe = false;
                }

                break;
            }

            // Resume the MAIN state if descent is detected again.
            if (abs(state_estimate.vertical_speed) > fsms_landed_detect_spd) {
                state = FSMState::STATE_MAIN;
                time_entered_cur_state = current_time;
            }

            break;
    }

    // Return the updated FSM state information.
    return state_data;
}