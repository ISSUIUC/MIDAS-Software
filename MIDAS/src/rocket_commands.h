#pragma once


//these will change the settings in telemetry_backend
struct btRadioSettings { 
    float freq;
    int signalBandwidth;
    int codingRate4;
    int spreadingFactor;
    bool payloadCRC;

};

//struct for thresholds
struct secondStageThresholds{
     // Transition to FIRST_BOOST if acceleration is greater than this
    double sustainer_idle_to_first_boost_acceleration_threshold;

    // Return state to IDLE if not boosting for this amount of time (ms)
    double sustainer_idle_to_first_boost_time_threshold;

    // Transition to SECOND_BOOST from SUSTAINER_IGNITION if acceleration greater than this
    double sustainer_ignition_to_second_boost_acceleration_threshold;

    // Return state to SECOND_BOOST if not boosting for this amount of time (ms)
    double sustainer_second_boost_to_coast_time_threshold;

    // Transition to COAST if acceleration is less than this value (g)
    double sustainer_coast_detection_acceleration_threshold;

    // Reach apogee state when vertical speed is less than or equal to this value
    double sustainer_coast_to_apogee_vertical_speed_threshold;

    // Revert back to COAST if apogee was too brief
    double sustainer_apogee_check_threshold;

    // Move on to DROGUE_DEPLOT after being in apogee for this amount of time
    double sustainer_apogee_timer_threshold;

    // Move on to DROGUE after a second of reaching apogee
    double sustainer_drogue_timer_threshold;

    // Move on to MAIN after passing this amount of time
    double sustainer_main_to_main_deploy_timer_threshold;

    // Height required to deploy the main parachutes
    double sustainer_main_deploy_altitude_threshold;

    // Return to SUSTAINER_IGNITION if not in SECOND_BOOST for this amount of time (ms)
    double sustainer_ignition_to_second_boost_time_threshold;

    // Transition straight to coast after a certain amount of time not detecting second stage boost
    double sustainer_ignition_to_coast_timer_threshold;

    // Revert back to main if the landed was too short
    double sustainer_landed_timer_threshold;

    // Return state to FIRST_BOOST if not in BURNOUT for this amount of time (ms)
    double sustainer_first_boost_to_burnout_time_threshold;

    // Transition to LANDED from MAIN if vertical speed is less than this threshold
    double sustainer_landed_vertical_speed_threshold;

    // Stores a small jerk value
    double sustainer_drogue_jerk_threshold;

    // Stores a small jerk value
    double sustainer_main_jerk_threshold;
};

struct rocketCommands {

    union{
        btRadioSettings settings;
        secondStageThresholds thresholds;
    } data;

};