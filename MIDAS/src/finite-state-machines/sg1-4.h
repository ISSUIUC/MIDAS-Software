#pragma once

// ----------------------------------
// SECOND STAGE THRESHOLDS
// ----------------------------------
struct SustainerThresholds {
    struct Idle {
        static constexpr int to_first_boost_acceleration_threshold = 3; // G
        static constexpr int to_first_boost_time_threshold = 1000; // ms
    } idle;

    struct Ignition {
        static constexpr int to_second_boost_acceleration_threshold = 3; // G
        static constexpr int to_second_boost_time_threshold = 1000; // ms
        static constexpr int to_coast_timer_threshold = 5000; // ms
    } ignition;

    struct SecondBoost {
        static constexpr int to_coast_time_threshold = 1000; // ms
    } second_boost;

    struct Coast {
        static constexpr double detection_acceleration_threshold = 0.2; // m/s^2
        static constexpr int to_apogee_vertical_speed_threshold = 15; // m/s
    } coast;

    struct Apogee {
        static constexpr int backto_coast_vertical_speed_threshold = 10; // m/s
        static constexpr int check_threshold = 1000; // ms
        static constexpr int timer_threshold = 1000; // ms
    } apogee;

    struct Drogue {
        static constexpr int timer_threshold = 3000; // ms
        static constexpr int jerk_threshold = 200; // m/s^3
    } drogue;

    struct Main {
        static constexpr int to_main_deploy_timer_threshold = 3000; // ms
        static constexpr int deploy_altitude_threshold = 1006; // m
        static constexpr int jerk_threshold = 300; // m/s^3
    } main;

    struct Landed {
        static constexpr int timer_threshold = 5000; // ms
        static constexpr int vertical_speed_threshold = 3; // m/s
    } landed;

    struct Burnout {
        static constexpr int to_first_boost_time_threshold = 1000; // ms
    } burnout;
};

// ----------------------------------
// FIRST STAGE THRESHOLDS
// ----------------------------------
struct BoosterThresholds {
    struct Idle {
        static constexpr int to_first_boost_acceleration_threshold = 3; // G
        static constexpr int to_first_boost_time_threshold = 1000; // ms
    } idle;

    struct FirstSeparation {
        static constexpr int time_threshold = 3000; // ms
        static constexpr int jerk_threshold = 300; // m/s^3
    } first_separation;

    struct Coast {
        static constexpr double detection_acceleration_threshold = 0.2; // G
        static constexpr int to_apogee_vertical_speed_threshold = 20; // m/s
    } coast;

    struct Apogee {
        static constexpr int check_threshold = 1000; // ms
        static constexpr int timer_threshold = 1000; // ms
    } apogee;

    struct Drogue {
        static constexpr int timer_threshold = 3000; // ms
        static constexpr int jerk_threshold = 200; // m/s^3
    } drogue;

    struct Main {
        static constexpr int to_main_deploy_timer_threshold = 3000; // ms
        static constexpr int deploy_altitude_threshold = 3000; // m
        static constexpr int jerk_threshold = 300; // m/s^3
    } main;

    struct Landed {
        static constexpr int timer_threshold = 5000; // ms
        static constexpr int vertical_speed_threshold = 4; // m/s
    } landed;

    struct Ignition {
        static constexpr int to_second_boost_time_threshold = 1000; // ms
        static constexpr int to_coast_timer_threshold = 5000; // ms
    } ignition;

    struct Burnout {
        static constexpr int to_first_boost_time_threshold = 1000; // ms
    } burnout;
};
