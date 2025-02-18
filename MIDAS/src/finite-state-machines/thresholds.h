#pragma once

// ----------------------------------
// SAFETY THRESHOLDS
// ----------------------------------

// Transition back to STATE_SAFE if this much time has passed without firing a pyro (ms)
#define safety_pyro_test_disarm_time 10000

// ----------------------------------
// SECOND STAGE THRESHOLDS
// ----------------------------------

// Regardless of sensor inputs, stay on pyro firing states for at LEAST this time. (ms)
#define sustainer_pyro_firing_time_minimum 100

// Transition to FIRST_BOOST if acceleration is greater than this (G)
#define sustainer_idle_to_first_boost_acceleration_threshold 3

// Return state to IDLE if not boosting for this amount of time (ms)
#define sustainer_idle_to_first_boost_time_threshold 1000

// Transition to SECOND_BOOST from SUSTAINER_IGNITION if acceleration greater than this (G)
#define sustainer_ignition_to_second_boost_acceleration_threshold 4

// Return state to SECOND_BOOST if not boosting for this amount of time (ms)
#define sustainer_second_boost_to_coast_time_threshold 1000

// Transition to COAST if acceleration is less than this value (g)
#define sustainer_coast_detection_acceleration_threshold 0.2

// Reach apogee state when vertical speed is less than or equal to this value (m/s)
#define sustainer_coast_to_apogee_vertical_speed_threshold 15

// Revert back to COAST if the vertical speed in apogee is too high (was 0 before which may have caused it keep jumping back to COAST) (m/s)
#define sustainer_apogee_backto_coast_vertical_speed_threshold 10

// Revert back to COAST if apogee was too brief (ms)
#define sustainer_apogee_check_threshold 1000

// Move on to DROGUE_DEPLOT after being in apogee for this amount of time (ms)
#define sustainer_apogee_timer_threshold 1000

// Move on to DROGUE after a second of reaching apogee (ms)
#define sustainer_drogue_timer_threshold 3000

// Move on to MAIN after passing this amount of time (ms)
#define sustainer_main_to_main_deploy_timer_threshold 3000

// Height required to deploy the main parachutes (m)
#define sustainer_main_deploy_altitude_threshold 1006

// The minimum delay between drogue deployment and main deployment
#define sustainer_main_deploy_delay_after_drogue 1000

// Return to SUSTAINER_IGNITION if not in SECOND_BOOST for this amount of time (ms)
#define sustainer_ignition_to_second_boost_time_threshold 1000

// Transition straight to coast after a certain amount of time not detecting second stage boost (ms)
#define sustainer_ignition_to_coast_timer_threshold 5000

// Revert back to main if the landed was too short (ms)
#define sustainer_landed_timer_threshold 5000

// Return state to FIRST_BOOST if not in BURNOUT for this amount of time (ms)
#define sustainer_coast_time 3000

// Transition to LANDED from MAIN if vertical speed is less than this threshold (m/s)
#define sustainer_landed_vertical_speed_threshold 4

// Lock out further transitions from LANDED after this much time passes in the LANDED state. (ms)
#define sustainer_landed_time_lockout 60000

// Prevent us from inadvertently entering the LANDED state when we're at a low velocity at main deploy. (ms)
#define sustainer_main_to_landed_lockout 5000

// Stores a small jerk value (m/s^3)
#define sustainer_drogue_jerk_threshold 200

// Stores a small jerk value (m/s^3)
#define sustainer_main_jerk_threshold 300


// ----------------------------------
// FIRST STAGE THRESHOLDS
// ----------------------------------

// Regardless of sensor inputs, stay on pyro firing states for at LEAST this time. (ms)
#define booster_pyro_firing_time_minimum 200

// Transition to FIRST_BOOST if acceleration is greater than this (G)
#define booster_idle_to_first_boost_acceleration_threshold 3

// Return state to IDLE if not boosting for this amount of time (ms)
#define booster_idle_to_first_boost_time_threshold 1000

// Move on regardless if it separates or not i.e. if state is FIRST_SEPERATION for over this amount of time (ms)
#define booster_first_seperation_time_threshold 3000

// Transition to COAST if acceleration is less than this value (g)
#define booster_coast_detection_acceleration_threshold 0.2

// Reach apogee state when vertical speed is less than or equal to this value (m/s)
#define booster_coast_to_apogee_vertical_speed_threshold 20

// Revert back to COAST if apogee was too brief (ms)
#define booster_apogee_check_threshold 1000

// Move on to DROGUE_DEPLOT after being in apogee for this amount of time (ms)
#define booster_apogee_timer_threshold 1000

// Move on to DROGUE after a second of reaching apogee (ms)
#define booster_drogue_timer_threshold 3000

// Move on to MAIN after passing this amount of time (ms)
#define booster_main_to_main_deploy_timer_threshold 3000

// Height required to deploy the main parachutes (m)
// [STARGAZER 1.4] This is a "dontcare" value --> The booster does not have a drogue, we transition immediately to MAIN
#define booster_main_deploy_altitude_threshold 999999

// The minimum delay between drogue deployment and main deployment
#define booster_main_deploy_delay_after_drogue 1000

// Return to SUSTAINER_IGNITION if not in SECOND_BOOST for this amount of time (ms)
#define booster_ignition_to_second_boost_time_threshold 1000

// Transition straight to coast after a certain amount of time not detecting second stage boost (ms)
#define booster_ignition_to_coast_timer_threshold 5000

// Revert back to main if the landed was too short (ms)
#define booster_landed_timer_threshold 5000

// Return state to FIRST_BOOST if not in BURNOUT for this amount of time (ms)
#define booster_first_boost_to_burnout_time_threshold 1000

// Transition to LANDED from MAIN if vertical speed is less than this threshold (G)
#define booster_landed_vertical_speed_threshold 4

// Lock out further transitions from LANDED after this much time passes in the LANDED state. (ms)
#define booster_landed_time_lockout 60000

// Prevent us from inadvertently entering the LANDED state when we're at a low velocity at main deploy. (ms)
#define booster_main_to_landed_lockout 5000

// Stores a small jerk value (m/s^3)
#define booster_first_separation_jerk_threshold 300

// Stores a small jerk value (m/s^3)
#define booster_drogue_jerk_threshold 200

// Stores a small jerk value (m/s^3)
#define booster_main_jerk_threshold 300
