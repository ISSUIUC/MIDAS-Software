#pragma once
#include <cmath>

// ----------------------------------
// SECOND STAGE THRESHOLDS
// ----------------------------------

// Transition to SECOND_BOOST from SUSTAINER_IGNITION if acceleration greater than this
static constexpr float sustainer_ignition_to_second_boost_acceleration_threshold = 3;

// Return state to SECOND_BOOST if not boosting for this amount of time (ms)
static constexpr float second_boost_to_coast_time_threshold = 1000;

// Move on to DROGUE_DEPLOT after being in apogee for this amount of time
static constexpr float apogee_timer_threshold = 1000;

// Move on to DROGUE after a second of reaching apogee
static constexpr float drogue_timer_threshold = 3000;

// Move on to MAIN after passing this amount of time
static constexpr float main_to_main_deploy_timer_threshold = 3000;

// Height required to deploy the main parachutes
static constexpr float main_deploy_altitude_threshold = 3000;

// Return to SUSTAINER_IGNITION if not in SECOND_BOOST for this amount of time (ms)
static constexpr float sustainer_ignition_to_second_boost_time_threshold = 1000;

// Transition straight to coast after a certain amount of time not detecting second stage boost
static constexpr float sustainer_ignition_to_coast_timer_threshold = 5000;

// ----------------------------------
// FIRST STAGE THRESHOLDS
// ----------------------------------



// ----------------------------------
// COMMON THRESHOLDS
// ----------------------------------

// Transition to FIRST_BOOST if acceleration is greater than this
static constexpr float idle_to_first_boost_acceleration_threshold = 3;

// Return state to IDLE if not boosting for this amount of time (ms)
static constexpr float idle_to_first_boost_time_threshold = 1000;

// Transition to COAST if acceleration is less than this value (g)
static constexpr float coast_detection_acceleration_threshold = 0;

// Reach apogee state when vertical speed is less than or equal to this value
static constexpr float coast_to_apogee_vertical_speed_threshold = 0;

// Revert back to COAST if apogee was too brief 
static constexpr float apogee_check_threshold = 1000;

// Revert back to main if the landed was too short
static constexpr float landed_timer_threshold = 5000;

// Return state to FIRST_BOOST if not in BURNOUT for this amount of time (ms)
static constexpr float first_boost_to_burnout_time_threshold = 1000;

// Transition to LANDED from MAIN if vertical speed is less than this threshold
static constexpr float landed_vertical_speed_threshold = 0;

// Move on regardless if it separates or not i.e. if state is FIRST_SEPERATION for over this amount of time (ms)
static constexpr float first_seperation_time_threshold = 3000;

// Stores a small jerk value 
static constexpr float jerk_threshold = 0;
