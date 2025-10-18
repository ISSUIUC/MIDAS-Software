#include "rocket_state.h"

#ifdef IS_SUSTAINER

// SAFE
void safe_to_pyroTest(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
}
void safe_to_stateIdle(CommandFlags& commands) {
    commands.should_transition_idle = false;
}

// PYRO TEST
void pyroTest_to_safe_forced(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
    commands.should_transition_idle = false;
    commands.should_transition_safe = false;
}
void pyroTest_to_safe_timed(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
}
void pyroTest_to_firstBoost(CommandFlags& commands) {
    
}

// IDLE
void idle_to_safe_forced(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
    commands.should_transition_idle = false;
    commands.should_transition_safe = false;
}

void idle_to_firstBoost(CommandFlags& commands) {
    commands.FSM_should_set_cam_feed_cam1 = true;
}

// FIRST BOOST
void firstBoost_to_idle(CommandFlags& commands) {}
void firstBoost_to_burnout(CommandFlags& commands) {}

// BURNOUT
void burnout_to_firstBoost(CommandFlags& commands) {}
void burnout_to_sustainerIgnition(CommandFlags& commands) {}
void burnout_to_firstSeparation(CommandFlags& commands) {}

// SUSTAINER IGNITION
void sustainerIgnition_to_coast(CommandFlags& commands) {}
void sustainerIgnition_to_secondBoost(CommandFlags& commands) {}

// FIRST SEPARATION
void firstSeparation_to_coast_jerk(CommandFlags& commands) {}
void firstSeparation_to_coast_timed(CommandFlags& commands) {}

// SECOND BOOST
void secondBoost_to_sustainerIgnition(CommandFlags& commands);
void secondBoost_to_coast(CommandFlags& commands);

// COAST
void coast_to_secondBoost(CommandFlags& commands);
void coast_to_apogee(CommandFlags& commands) {
    commands.FSM_should_swap_camera_feed = true;
}

// APOGEE
void apogee_to_coast(CommandFlags& commands);
void apogee_to_drogueDeploy(CommandFlags& commands);

// DROUGE DEPLOY
void drogueDeploy_to_drogue_jerk(CommandFlags& commands);
void drogueDeploy_to_drouge_timed(CommandFlags& commands);

// DROGUE
void drogue_to_mainDeploy(CommandFlags& commands);

// MAIN DEPLOY
void mainDeploy_to_main_jerk(CommandFlags& commands);
void mainDeploy_to_main_timed(CommandFlags& commands);

// MAIN
void main_to_landed(CommandFlags& commands);

// LANDED
void landed_to_safe(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
    commands.should_transition_idle = false;
    commands.should_transition_safe = false;
}
void landed_to_main(CommandFlags& commands);

#else

// SAFE
void safe_to_pyroTest(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
}
void safe_to_stateIdle(CommandFlags& commands) {
    commands.should_transition_idle = false;
}

// PYRO TEST
void pyroTest_to_safe_forced(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
    commands.should_transition_idle = false;
    commands.should_transition_safe = false;
}
void pyroTest_to_safe_timed(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
}
void pyroTest_to_firstBoost(CommandFlags& commands) {
    
}

// IDLE
void idle_to_safe_forced(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
    commands.should_transition_idle = false;
    commands.should_transition_safe = false;
}

void idle_to_firstBoost(CommandFlags& commands) {
    commands.FSM_should_set_cam_feed_cam1 = true;
}

// FIRST BOOST
void firstBoost_to_idle(CommandFlags& commands) {}
void firstBoost_to_burnout(CommandFlags& commands) {}

// BURNOUT
void burnout_to_firstBoost(CommandFlags& commands) {}
void burnout_to_sustainerIgnition(CommandFlags& commands) {}
void burnout_to_firstSeparation(CommandFlags& commands) {}

// SUSTAINER IGNITION
void sustainerIgnition_to_coast(CommandFlags& commands) {}
void sustainerIgnition_to_secondBoost(CommandFlags& commands) {}

// FIRST SEPARATION
void firstSeparation_to_coast_jerk(CommandFlags& commands) {}
void firstSeparation_to_coast_timed(CommandFlags& commands) {}

// SECOND BOOST
void secondBoost_to_sustainerIgnition(CommandFlags& commands);
void secondBoost_to_coast(CommandFlags& commands);

// COAST
void coast_to_secondBoost(CommandFlags& commands);
void coast_to_apogee(CommandFlags& commands) {
    commands.FSM_should_swap_camera_feed = true;
}

// APOGEE
void apogee_to_coast(CommandFlags& commands);
void apogee_to_drogueDeploy(CommandFlags& commands);

// DROUGE DEPLOY
void drogueDeploy_to_drogue_jerk(CommandFlags& commands);
void drogueDeploy_to_drouge_timed(CommandFlags& commands);

// DROGUE
void drogue_to_mainDeploy(CommandFlags& commands);

// MAIN DEPLOY
void mainDeploy_to_main_jerk(CommandFlags& commands);
void mainDeploy_to_main_timed(CommandFlags& commands);

// MAIN
void main_to_landed(CommandFlags& commands);

// LANDED
void landed_to_safe(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
    commands.should_transition_idle = false;
    commands.should_transition_safe = false;
}
void landed_to_main(CommandFlags& commands);

#endif