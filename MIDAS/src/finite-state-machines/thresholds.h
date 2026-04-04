#pragma once

// (ms)  Amount of time in PYRO_TEST spent before auto-transitioning to SAFE
#define fsms_pt_disarm_t 10000

// (G)   Acceleration that must be sustained to detect a motor burn
#define fsms_boost_xl 3

// (ms)  Amount of time that a boost must be sustained to register as a motor burn
#define fsms_boost_lockin_t 1000

// (G)   Acceleration threshold for detecting motor burnout (signed).
#define fsms_burnout_xl 0.2

// (ms)  Amount of time that burnout needs to be detected to lock in the state
#define fsms_burnout_lockin_t 200

// (m/s) Vertical velocity (signed) at which the apogee detection can begin.
#define fsms_apogee_detect_spd 25

// (ms)  Amount of time that apogee detect must be triggered to transition to the DROGUE state
#define fsms_apogee_lockin_t 500

// (ms)  Minimum amount of time that has to pass in the DROGUE state to be able to transition to MAIN 
//       (Configure this value to prevent overpressure events)
#define fsms_main_lockout_t 1000

// (ms)  Time for which the LANDED criteria have to be met to prevent back-transition into MAIN
#define fsms_landed_t 5000

// (m/s) The speed at which LANDED detection can begin.
#define fsms_landed_detect_spd 3

// (ms)  Time lockout after MAIN entry to prevent early LANDED detection
#define fsms_landed_t_lockout 3000

// (m/s) Velocity at which the cruise lockout enables for apogee detection
#define fsms_cruise_lockout_spd 250