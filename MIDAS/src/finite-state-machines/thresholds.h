#pragma once
#include <cmath>
// Transition from boost to coast if low acceleration detected for 300 ms
static constexpr float launch_detection_acceleration_threshold = 3;

static constexpr float first_separation_linear_acceleration_thresh = 3;

// Return state to IDLE if not boosting for this amount of time (ms)
static constexpr float idle_to_first_boost_time_threshold = 1000;

// Transition to COAST if acceleration is greater than this value (g)
static constexpr float coast_detection_acceleration_threshold = 0;

// Return state to SECOND_BOOST if not boosting for this amount of time (ms)
static constexpr float second_boost_to_coast_time_threshold = 1000;

// Move on to drouge deploy after a second of reaching apogee
static constexpr float apogee_timer_threshold = 1000;

// Move on to drouge deploy after a second of reaching apogee
static constexpr float drogue_timer_threshold = 3000;

// Move on to drouge deploy after a second of reaching apogee
static constexpr float main_timer_threshold = 3000;