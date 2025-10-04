#include "rocket_state.h"


// SAFE
void safe_to_pyroTest();
void safe_to_stateIdle();

// PYRO TEST
void pyroTest_to_safe_forced();
void pyroTest_to_safe_timed();
void pyroTest_to_firstBoost();

// FIRST BOOST
void firstBoost_to_idle();
void firstBoost_to_burnout();

// BURNOUT
void burnout_to_firstBoost();
void burnout_to_sustainerIgnition();

// SUSTAINER IGNITION
void sustainerIgnition_to_coast();
void sustainerIgnition_to_secondBoost();

// SECOND BOOST
void secondBoost_to_sustainerIgnition();
void secondBoost_to_coast();

// COAST
void coast_to_secondBoost();
void coast_to_apogee();

// APOGEE
void apogee_to_coast();
void apogee_to_drogueDeploy();

// DROUGE DEPLOY
void drogueDeploy_to_drogue_jerk();
void drogueDeploy_to_drouge_timed();

// DROGUE
void drogue_to_mainDeploy();

// MAIN DEPLOY
void mainDeploy_to_main_jerk();
void mainDeploy_to_main_timed();

// MAIN
void main_to_landed();

// LANDED
void landed_to_safe();
void landed_to_main();