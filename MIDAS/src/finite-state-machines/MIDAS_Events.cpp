#include "MIDAS_Events.h"

#ifdef IS_SUSTAINER

// SAFE
void MIDAS_Events::safe_to_pyroTest(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
}
void MIDAS_Events::safe_to_stateIdle(CommandFlags& commands) {
    commands.should_transition_idle = false;
}

// PYRO TEST
void MIDAS_Events::pyroTest_to_safe_forced(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
    commands.should_transition_idle = false;
    commands.should_transition_safe = false;
}
void MIDAS_Events::pyroTest_to_safe_timed(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
}
void MIDAS_Events::pyroTest_to_firstBoost(CommandFlags& commands) {
    
}

// IDLE
void MIDAS_Events::idle_to_safe_forced(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
    commands.should_transition_idle = false;
    commands.should_transition_safe = false;
}

void MIDAS_Events::idle_to_firstBoost(CommandFlags& commands) {
    commands.FSM_should_set_cam_feed_cam1 = true;
}

// FIRST BOOST
void MIDAS_Events::firstBoost_to_idle(CommandFlags& commands) {}
void MIDAS_Events::firstBoost_to_burnout(CommandFlags& commands) {}

// BURNOUT
void MIDAS_Events::burnout_to_firstBoost(CommandFlags& commands) {}
void MIDAS_Events::burnout_to_sustainerIgnition(CommandFlags& commands) {}
void MIDAS_Events::burnout_to_firstSeparation(CommandFlags& commands) {}

// SUSTAINER IGNITION
void MIDAS_Events::sustainerIgnition_to_coast(CommandFlags& commands) {}
void MIDAS_Events::sustainerIgnition_to_secondBoost(CommandFlags& commands) {}

// FIRST SEPARATION
void MIDAS_Events::firstSeparation_to_coast_jerk(CommandFlags& commands) {}
void MIDAS_Events::firstSeparation_to_coast_timed(CommandFlags& commands) {}

// SECOND BOOST
void MIDAS_Events::secondBoost_to_sustainerIgnition(CommandFlags& commands) {}
void MIDAS_Events::secondBoost_to_coast(CommandFlags& commands) {}

// COAST
void MIDAS_Events::coast_to_secondBoost(CommandFlags& commands) {}
void MIDAS_Events::coast_to_apogee(CommandFlags& commands) {
    commands.FSM_should_swap_camera_feed = true;
}

// APOGEE
void MIDAS_Events::apogee_to_coast(CommandFlags& commands) {}
void MIDAS_Events::apogee_to_drogueDeploy(CommandFlags& commands) {}

// DROUGE DEPLOY
void MIDAS_Events::drogueDeploy_to_drogue_jerk(CommandFlags& commands) {}
void MIDAS_Events::drogueDeploy_to_drouge_timed(CommandFlags& commands) {}

// DROGUE
void MIDAS_Events::drogue_to_mainDeploy(CommandFlags& commands) {}

// MAIN DEPLOY
void MIDAS_Events::mainDeploy_to_main_jerk(CommandFlags& commands) {}
void MIDAS_Events::mainDeploy_to_main_timed(CommandFlags& commands) {}

// MAIN
void MIDAS_Events::main_to_landed(CommandFlags& commands) {}

// LANDED
void MIDAS_Events::landed_to_safe(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
    commands.should_transition_idle = false;
    commands.should_transition_safe = false;
}
void MIDAS_Events::landed_to_main(CommandFlags& commands) {}

#else

// SAFE
void MIDAS_Events::safe_to_pyroTest(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
}
void MIDAS_Events::safe_to_stateIdle(CommandFlags& commands) {
    commands.should_transition_idle = false;
}

// PYRO TEST
void MIDAS_Events::pyroTest_to_safe_forced(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
    commands.should_transition_idle = false;
    commands.should_transition_safe = false;
}
void MIDAS_Events::pyroTest_to_safe_timed(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
}
void MIDAS_Events::pyroTest_to_firstBoost(CommandFlags& commands) {
    
}

// IDLE
void MIDAS_Events::idle_to_safe_forced(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
    commands.should_transition_idle = false;
    commands.should_transition_safe = false;
}

void MIDAS_Events::idle_to_firstBoost(CommandFlags& commands) {
    commands.FSM_should_set_cam_feed_cam1 = true;
}

// FIRST BOOST
void MIDAS_Events::firstBoost_to_idle(CommandFlags& commands) {}
void MIDAS_Events::firstBoost_to_burnout(CommandFlags& commands) {}

// BURNOUT
void MIDAS_Events::burnout_to_firstBoost(CommandFlags& commands) {}
void MIDAS_Events::burnout_to_sustainerIgnition(CommandFlags& commands) {}
void MIDAS_Events::burnout_to_firstSeparation(CommandFlags& commands) {}

// SUSTAINER IGNITION
void MIDAS_Events::sustainerIgnition_to_coast(CommandFlags& commands) {}
void MIDAS_Events::sustainerIgnition_to_secondBoost(CommandFlags& commands) {}

// FIRST SEPARATION
void MIDAS_Events::firstSeparation_to_coast_jerk(CommandFlags& commands) {}
void MIDAS_Events::firstSeparation_to_coast_timed(CommandFlags& commands) {}

// SECOND BOOST
void MIDAS_Events::secondBoost_to_sustainerIgnition(CommandFlags& commands) {}
void MIDAS_Events::secondBoost_to_coast(CommandFlags& commands) {}

// COAST
void MIDAS_Events::coast_to_secondBoost(CommandFlags& commands) {}
void MIDAS_Events::coast_to_apogee(CommandFlags& commands) {}

// APOGEE
void MIDAS_Events::apogee_to_coast(CommandFlags& commands) {}
void MIDAS_Events::apogee_to_drogueDeploy(CommandFlags& commands) {}

// DROUGE DEPLOY
void MIDAS_Events::drogueDeploy_to_drogue_jerk(CommandFlags& commands) {}
void MIDAS_Events::drogueDeploy_to_drouge_timed(CommandFlags& commands) {}

// DROGUE
void MIDAS_Events::drogue_to_mainDeploy(CommandFlags& commands) {}

// MAIN DEPLOY
void MIDAS_Events::mainDeploy_to_main_jerk(CommandFlags& commands) {}
void MIDAS_Events::mainDeploy_to_main_timed(CommandFlags& commands) {}

// MAIN
void MIDAS_Events::main_to_landed(CommandFlags& commands) {}

// LANDED
void MIDAS_Events::landed_to_safe(CommandFlags& commands) {
    commands.should_transition_pyro_test = false;
    commands.should_transition_idle = false;
    commands.should_transition_safe = false;
}
void MIDAS_Events::landed_to_main(CommandFlags& commands) {}

#endif