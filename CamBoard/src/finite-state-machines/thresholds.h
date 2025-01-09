#pragma once

// ----------------------------------
// SECOND STAGE THRESHOLDS
// ----------------------------------

// Transition to FIRST_BOOST if acceleration is greater than this
#define sustainer_idle_to_first_boost_acceleration_threshold 3

// Return state to IDLE if not boosting for this amount of time (ms)
#define sustainer_idle_to_first_boost_time_threshold 1000

// Transition to SECOND_BOOST from SUSTAINER_IGNITION if acceleration greater than this
#define sustainer_ignition_to_second_boost_acceleration_threshold 3

// Return state to SECOND_BOOST if not boosting for this amount of time (ms)
#define sustainer_second_boost_to_coast_time_threshold 1000

// Transition to COAST if acceleration is less than this value (g)
#define sustainer_coast_detection_acceleration_threshold 0.2

// Reach apogee state when vertical speed is less than or equal to this value
#define sustainer_coast_to_apogee_vertical_speed_threshold 15

// Revert back to COAST if the vertical speed in apogee is too high (was 0 before which may have caused it keep jumping back to COAST)
#define sustainer_apogee_backto_coast_vertical_speed_threshold 10

// Revert back to COAST if apogee was too brief
#define sustainer_apogee_check_threshold 1000

// Move on to DROGUE_DEPLOT after being in apogee for this amount of time
#define sustainer_apogee_timer_threshold 1000

// Move on to DROGUE after a second of reaching apogee
#define sustainer_drogue_timer_threshold 3000

// Move on to MAIN after passing this amount of time
#define sustainer_main_to_main_deploy_timer_threshold 3000

// Height required to deploy the main parachutes
#define sustainer_main_deploy_altitude_threshold 3000

// Return to SUSTAINER_IGNITION if not in SECOND_BOOST for this amount of time (ms)
#define sustainer_ignition_to_second_boost_time_threshold 1000

// Transition straight to coast after a certain amount of time not detecting second stage boost
#define sustainer_ignition_to_coast_timer_threshold 5000

// Revert back to main if the landed was too short
#define sustainer_landed_timer_threshold 5000

// Return state to FIRST_BOOST if not in BURNOUT for this amount of time (ms)
#define sustainer_first_boost_to_burnout_time_threshold 1000

// Transition to LANDED from MAIN if vertical speed is less than this threshold
#define sustainer_landed_vertical_speed_threshold 20

// Stores a small jerk value
#define sustainer_drogue_jerk_threshold 200

// Stores a small jerk value
#define sustainer_main_jerk_threshold 300


// ----------------------------------
// FIRST STAGE THRESHOLDS
// ----------------------------------

// Transition to FIRST_BOOST if acceleration is greater than this
#define booster_idle_to_first_boost_acceleration_threshold 3

// Return state to IDLE if not boosting for this amount of time (ms)
#define booster_idle_to_first_boost_time_threshold 1000

// Move on regardless if it separates or not i.e. if state is FIRST_SEPERATION for over this amount of time (ms)
#define booster_first_seperation_time_threshold 3000

// Transition to COAST if acceleration is less than this value (g)
#define booster_coast_detection_acceleration_threshold 0.2

// Reach apogee state when vertical speed is less than or equal to this value
#define booster_coast_to_apogee_vertical_speed_threshold 20

// Revert back to COAST if apogee was too brief
#define booster_apogee_check_threshold 1000

// Move on to DROGUE_DEPLOT after being in apogee for this amount of time
#define booster_apogee_timer_threshold 1000

// Move on to DROGUE after a second of reaching apogee
#define booster_drogue_timer_threshold 3000

// Move on to MAIN after passing this amount of time
#define booster_main_to_main_deploy_timer_threshold 3000

// Height required to deploy the main parachutes
#define booster_main_deploy_altitude_threshold 3000
// Return to SUSTAINER_IGNITION if not in SECOND_BOOST for this amount of time (ms)
#define booster_ignition_to_second_boost_time_threshold 1000

// Transition straight to coast after a certain amount of time not detecting second stage boost
#define booster_ignition_to_coast_timer_threshold 5000

// Revert back to main if the landed was too short
#define booster_landed_timer_threshold 5000

// Return state to FIRST_BOOST if not in BURNOUT for this amount of time (ms)
#define booster_first_boost_to_burnout_time_threshold 1000

// Transition to LANDED from MAIN if vertical speed is less than this threshold
#define booster_landed_vertical_speed_threshold 20

// Stores a small jerk value
#define booster_first_separation_jerk_threshold 300

// Stores a small jerk value
#define booster_drogue_jerk_threshold 200

// Stores a small jerk value
#define booster_main_jerk_threshold 300