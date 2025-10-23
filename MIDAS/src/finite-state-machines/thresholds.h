#pragma once

// FSM Threshold file: This file stores all values referenced by the MIDAS FSM in ./fsm.cpp
// Tags: @SDA and @REC tags indicate that these values are of interest to these teams, and may optionally include a note.
// (Core Setting) -- This is a value that is set by general flight profiles and should not change based solely on new flight profiles
// (Flight Parameter) -- This is a value that is expected to change between every flight.
// (UNUSED) -- This value is not referenced in the MIDAS FSM and may be deleted

// ----------------------------------
// SAFETY THRESHOLDS
// ----------------------------------

// Transition back to STATE_SAFE if this much time has passed without firing a pyro (ms)
// (Core Setting)
#define safety_pyro_test_disarm_time 10000

// ----------------------------------
// SECOND STAGE THRESHOLDS
// ----------------------------------

// Regardless of sensor inputs, stay on pyro firing states for at LEAST this time. (ms)
// (Core Setting)
#define sustainer_pyro_firing_time_minimum 100

// Transition to FIRST_BOOST if acceleration is greater than this (G)
// @SDA: This value should be a reasonable lower bound experienced by the vehicle during the first stage boost.
//       Given values should err on the lower side if possible, but should never be below 2
// (Flight Parameter)
#define sustainer_idle_to_first_boost_acceleration_threshold 5

// Return state to IDLE if not boosting for this amount of time (ms)
// Note: This number should be as low as possible without causing false positives. Ultra-short boosts may break this value
// (Core Setting)
#define sustainer_idle_to_first_boost_time_threshold 1000

// Transition to SECOND_BOOST from SUSTAINER_IGNITION if acceleration greater than this (G)
// @SDA: This value should be a reasonable lower bound experienced by the vehicle during the sustainer stage boost.
//       Given values should err on the lower side if possible, but should never be below 2
// (Flight Parameter)
#define sustainer_ignition_to_second_boost_acceleration_threshold 8

// Return state to SECOND_BOOST if not boosting for this amount of time (ms)
// Note: This number should be as low as possible without causing false positives. Ultra-short boosts may break this value
// (Core Setting)
#define sustainer_second_boost_to_coast_time_threshold 1000

// Transition to COAST if acceleration is less than this value (g)
// @SDA (note): This is the absolute value of the magnitude of acceleration under which the system detects coast.
//              This value does not currently support negative values, and very small thresholds will likely break.
// (Core Setting)
#define sustainer_coast_detection_acceleration_threshold 0.2

// Reach apogee state when vertical speed is less than or equal to this value (m/s)
// (Core Setting)
#define sustainer_coast_to_apogee_vertical_speed_threshold 25

// Check vertical speed for this amount of time in the APOGEE state, move back to COAST if needed (m/s)
// (Core Setting)
#define sustainer_apogee_backto_coast_vertical_speed_threshold 25

// Revert back to COAST if the conditions for apogee are met for too little time (ms)
// (Core Setting)
#define sustainer_apogee_check_threshold 500

// Delay between the APOGEE state being confirmed and moving into DROGUE_DEPLOY for pyros (ms)
// (Core Setting)
#define sustainer_apogee_timer_threshold 500

// If drogue deployment is not auto-detected, transition to DROGUE anyway after this amount of time (ms)
// (Core Setting)
#define sustainer_drogue_timer_threshold 3000

// If main deployment is not auto-detected, transition to MAIN anyway after this amount of time (ms)
// (Core Setting)
#define sustainer_main_to_main_deploy_timer_threshold 3000

// Height ASL when the MAIN charge should be fired (m)
// @REC: MIDAS does not currently auto-calculate launch site altitudes. 
//       This value should always be set at (launch site ASL) + (desired altitude AGL)
// (Flight Parameter)
#define sustainer_main_deploy_altitude_threshold 545

// The minimum delay between drogue deployment and main deployment (ms)
// @REC: This is a safety feature to prevent overpressurization or internal firing, and forces MAIN_DEPLOY to wait at least this much time,
//       even if the drogue deployment happens under the main deployment altitude.
// (Flight Parameter)
#define sustainer_main_deploy_delay_after_drogue 1000

// Return to SUSTAINER_IGNITION if acceleration threshold for SECOND_BOOST is not met for this time (ms)
// (Core Setting)
#define sustainer_ignition_to_second_boost_time_threshold 1000

// Fallback to off-nominal coast if sustainer ignition is not confirmed for this amount of time (ms)
// @SDA: Ensure this value is long enough to account for delaying effects such as motor pressurization
// (Flight Parameter)
#define sustainer_ignition_to_coast_timer_threshold 5000

// Revert back to the MAIN state if the vehicle doesn't meet LANDED criteria for long enough (ms)
// (Core Setting)
#define sustainer_landed_timer_threshold 5000

// Return state to FIRST_BOOST if not in BURNOUT for this amount of time (ms)
// @SDA: This is the amount of time the sustainer should coast before attempting a second stage ignition
// (Flight Parameter)
#define sustainer_coast_time 3000

// Transition to LANDED from MAIN if vertical speed is less than this threshold (m/s)
// @REC: This number should be below the MINIMUM descent velocity of the vehicle with significant margin.
// (Flight Parameter)
#define sustainer_landed_vertical_speed_threshold 1

// Transition back to MAIN if vertical speed is greater than this threshold (m/s)
// @REC: This number should be below the MINIMUM descent velocity of the vehicle with significant margin.
// (Flight Parameter)
#define sustainer_landed_to_main_vertical_speed_threshold 3

// Lock out further transitions from LANDED after this much time passes in the LANDED state. (ms)
// (Core Setting)
#define sustainer_landed_time_lockout 60000

// Prevent us from inadvertently entering the LANDED state when we're at a low velocity at main deploy. (ms)
// (Core Setting)
#define sustainer_main_to_landed_lockout 5000

// The minimum expected jerk for a drogue deployment event (m/s^3)
// (Core Setting)
#define sustainer_drogue_jerk_threshold 200

// The minimum expected jerk for a main deployment event (m/s^3)
// (Core Setting)
#define sustainer_main_jerk_threshold 300


// ----------------------------------
// FIRST STAGE THRESHOLDS
// ----------------------------------

// Regardless of sensor inputs, stay on pyro firing states for at LEAST this time. (ms)
// (Core Setting)
#define booster_pyro_firing_time_minimum 100

// Transition to FIRST_BOOST if acceleration is greater than this (G)
// @SDA: This value should be a reasonable lower bound experienced by the vehicle during the first stage boost.
//       Given values should err on the lower side if possible, but should never be below 2
// (Flight Parameter)
#define booster_idle_to_first_boost_acceleration_threshold 5

// Return state to IDLE if not boosting for this amount of time (ms)
// Note: This number should be as low as possible without causing false positives. Ultra-short boosts may break this value
// (Core Setting)
#define booster_idle_to_first_boost_time_threshold 1000

// Continue into the COAST state even if the stage separation jerk is not detected properly after this time (ms)
// (Core Setting)
#define booster_first_seperation_time_threshold 3000

// Transition to COAST if acceleration is less than this value (g)
// @SDA (note): This is the absolute value of the magnitude of acceleration under which the system detects coast.
//              This value does not currently support negative values, and very small thresholds will likely break.
// (Core Setting)
#define booster_coast_detection_acceleration_threshold 0.2

// Reach apogee state when vertical speed is less than or equal to this value (m/s)
// (Core Setting)
#define booster_coast_to_apogee_vertical_speed_threshold 20

// Check vertical speed for this amount of time in the APOGEE state, move back to COAST if needed (m/s)
// (Core Setting)
#define booster_apogee_check_threshold 500

// Delay between the APOGEE state being confirmed and moving into DROGUE_DEPLOY for pyros (ms)
// (Core Setting)
#define booster_apogee_timer_threshold 500

// If drogue deployment is not auto-detected, transition to DROGUE anyway after this amount of time (ms)
// (Core Setting)
#define booster_drogue_timer_threshold 3000

// If main deployment is not auto-detected, transition to MAIN anyway after this amount of time (ms)
// (Core Setting)
#define booster_main_to_main_deploy_timer_threshold 3000

// Height ASL when the MAIN charge should be fired (m)
// @REC: MIDAS does not currently auto-calculate launch site altitudes. 
//       This value should always be set at (launch site ASL) + (desired altitude AGL)
// (Flight Parameter)
// [STARGAZER 1.4] This is a "dontcare" value --> The booster does not have a drogue, we transition immediately to MAIN
#define booster_main_deploy_altitude_threshold 545

// The minimum delay between drogue deployment and main deployment (ms)
// @REC: This is a safety feature to prevent overpressurization or internal firing, and forces MAIN_DEPLOY to wait at least this much time,
//       even if the drogue deployment happens under the main deployment altitude.
// (Flight Parameter)
#define booster_main_deploy_delay_after_drogue 1000

// Return to SUSTAINER_IGNITION if not in SECOND_BOOST for this amount of time (ms)
// (UNUSED)
#define booster_ignition_to_second_boost_time_threshold 1000

// Transition straight to coast after a certain amount of time not detecting second stage boost (ms)
// (UNUSED)
#define booster_ignition_to_coast_timer_threshold 5000

// Revert back to the MAIN state if the vehicle doesn't meet LANDED criteria for long enough (ms)
// (Core Setting)
#define booster_landed_timer_threshold 5000

// Amount of time after booster burn that the stage separation event should trigger (ms)
// @SDA
// (Flight parameter)
#define booster_first_boost_to_burnout_time_threshold 1000

// Transition to LANDED from MAIN if vertical speed is less than this threshold (m/s)
// @REC: This number should be below the MINIMUM descent velocity of the vehicle with significant margin.
// (Flight Parameter)
#define booster_landed_vertical_speed_threshold 1

// Transition back to MAIN if vertical speed is greater than this threshold (m/s)
// @REC: This number should be below the MINIMUM descent velocity of the vehicle with significant margin.
// (Flight Parameter)
#define booster_landed_to_main_vertical_speed_threshold 5

// Lock out further transitions from LANDED after this much time passes in the LANDED state. (ms)
// (Core Setting)
#define booster_landed_time_lockout 60000

// Prevent us from inadvertently entering the LANDED state when we're at a low velocity at main deploy. (ms)
// (Core Setting)
#define booster_main_to_landed_lockout 5000

// The minimum expected jerk to detect stage separation (m/s^3)
// (Core Setting)
#define booster_first_separation_jerk_threshold 300

// The minimum expected jerk for a drogue deployment event (m/s^3)
// (Core Setting)
#define booster_drogue_jerk_threshold 200

// The minimum expected jerk for a main deployment event (m/s^3)
// (Core Setting)
#define booster_main_jerk_threshold 300