#include "MIDAS_Events.h"

#ifdef IS_SUSTAINER

// SAFE
void MIDAS_Events::safe_to_pyroTest(RocketData& rocket_data) {
    rocket_data.command_flags.should_transition_pyro_test = false;
}
void MIDAS_Events::safe_to_stateIdle(RocketData& rocket_data) {
    rocket_data.command_flags.should_transition_idle = false;
}

// PYRO TEST
void MIDAS_Events::pyroTest_to_safe_forced(RocketData& rocket_data) {
    rocket_data.command_flags.should_transition_pyro_test = false;
    rocket_data.command_flags.should_transition_idle = false;
    rocket_data.command_flags.should_transition_safe = false;
}
void MIDAS_Events::pyroTest_to_safe_timed(RocketData& rocket_data) {
    rocket_data.command_flags.should_transition_pyro_test = false;
}
void MIDAS_Events::pyroTest_to_firstBoost(RocketData& rocket_data) {
    
}

// IDLE
void MIDAS_Events::idle_to_safe_forced(RocketData& rocket_data) {
    rocket_data.command_flags.should_transition_pyro_test = false;
    rocket_data.command_flags.should_transition_idle = false;
    rocket_data.command_flags.should_transition_safe = false;
}

void MIDAS_Events::idle_to_firstBoost(RocketData& rocket_data) {
    rocket_data.command_flags.FSM_should_set_cam_feed_cam1 = true;
}

// FIRST BOOST
void MIDAS_Events::firstBoost_to_idle(RocketData& rocket_data) {}
void MIDAS_Events::firstBoost_to_burnout(RocketData& rocket_data) {}

// BURNOUT
void MIDAS_Events::burnout_to_firstBoost(RocketData& rocket_data) {}
void MIDAS_Events::burnout_to_sustainerIgnition(RocketData& rocket_data) {}
void MIDAS_Events::burnout_to_firstSeparation(RocketData& rocket_data) {}

// SUSTAINER IGNITION
void MIDAS_Events::sustainerIgnition_to_coast(RocketData& rocket_data) {}
void MIDAS_Events::sustainerIgnition_to_secondBoost(RocketData& rocket_data) {}

// FIRST SEPARATION
void MIDAS_Events::firstSeparation_to_coast_jerk(RocketData& rocket_data) {}
void MIDAS_Events::firstSeparation_to_coast_timed(RocketData& rocket_data) {}

// SECOND BOOST
void MIDAS_Events::secondBoost_to_sustainerIgnition(RocketData& rocket_data) {}
void MIDAS_Events::secondBoost_to_coast(RocketData& rocket_data) {}

// COAST
void MIDAS_Events::coast_to_secondBoost(RocketData& rocket_data) {}
void MIDAS_Events::coast_to_apogee(RocketData& rocket_data) {
    rocket_data.command_flags.FSM_should_swap_camera_feed = true;
}

// APOGEE
void MIDAS_Events::apogee_to_coast(RocketData& rocket_data) {}
void MIDAS_Events::apogee_to_drogueDeploy(RocketData& rocket_data) {}

// DROUGE DEPLOY
void MIDAS_Events::drogueDeploy_to_drogue_jerk(RocketData& rocket_data) {}
void MIDAS_Events::drogueDeploy_to_drouge_timed(RocketData& rocket_data) {}

// DROGUE
void MIDAS_Events::drogue_to_mainDeploy(RocketData& rocket_data) {}

// MAIN DEPLOY
void MIDAS_Events::mainDeploy_to_main_jerk(RocketData& rocket_data) {}
void MIDAS_Events::mainDeploy_to_main_timed(RocketData& rocket_data) {}

// MAIN
void MIDAS_Events::main_to_landed(RocketData& rocket_data) {}

// LANDED
void MIDAS_Events::landed_to_safe(RocketData& rocket_data) {
    rocket_data.command_flags.should_transition_pyro_test = false;
    rocket_data.command_flags.should_transition_idle = false;
    rocket_data.command_flags.should_transition_safe = false;
}
void MIDAS_Events::landed_to_main(RocketData& rocket_data) {}

#else

// SAFE
void MIDAS_Events::safe_to_pyroTest(RocketData& rocket_data) {
    rocket_data.command_flags.should_transition_pyro_test = false;
}
void MIDAS_Events::safe_to_stateIdle(RocketData& rocket_data) {
    rocket_data.command_flags.should_transition_idle = false;
}

// PYRO TEST
void MIDAS_Events::pyroTest_to_safe_forced(RocketData& rocket_data) {
    rocket_data.command_flags.should_transition_pyro_test = false;
    rocket_data.command_flags.should_transition_idle = false;
    rocket_data.command_flags.should_transition_safe = false;
}
void MIDAS_Events::pyroTest_to_safe_timed(RocketData& rocket_data) {
    rocket_data.command_flags.should_transition_pyro_test = false;
}
void MIDAS_Events::pyroTest_to_firstBoost(RocketData& rocket_data) {
    
}

// IDLE
void MIDAS_Events::idle_to_safe_forced(RocketData& rocket_data) {
    rocket_data.command_flags.should_transition_pyro_test = false;
    rocket_data.command_flags.should_transition_idle = false;
    rocket_data.command_flags.should_transition_safe = false;
}

void MIDAS_Events::idle_to_firstBoost(RocketData& rocket_data) {
    rocket_data.command_flags.FSM_should_set_cam_feed_cam1 = true;
}

// FIRST BOOST
void MIDAS_Events::firstBoost_to_idle(RocketData& rocket_data) {}
void MIDAS_Events::firstBoost_to_burnout(RocketData& rocket_data) {}

// BURNOUT
void MIDAS_Events::burnout_to_firstBoost(RocketData& rocket_data) {}
void MIDAS_Events::burnout_to_sustainerIgnition(RocketData& rocket_data) {}
void MIDAS_Events::burnout_to_firstSeparation(RocketData& rocket_data) {}

// SUSTAINER IGNITION
void MIDAS_Events::sustainerIgnition_to_coast(RocketData& rocket_data) {}
void MIDAS_Events::sustainerIgnition_to_secondBoost(RocketData& rocket_data) {}

// FIRST SEPARATION
void MIDAS_Events::firstSeparation_to_coast_jerk(RocketData& rocket_data) {}
void MIDAS_Events::firstSeparation_to_coast_timed(RocketData& rocket_data) {}

// SECOND BOOST
void MIDAS_Events::secondBoost_to_sustainerIgnition(RocketData& rocket_data) {}
void MIDAS_Events::secondBoost_to_coast(RocketData& rocket_data) {}

// COAST
void MIDAS_Events::coast_to_secondBoost(RocketData& rocket_data) {}
void MIDAS_Events::coast_to_apogee(RocketData& rocket_data) {}

// APOGEE
void MIDAS_Events::apogee_to_coast(RocketData& rocket_data) {}
void MIDAS_Events::apogee_to_drogueDeploy(RocketData& rocket_data) {}

// DROUGE DEPLOY
void MIDAS_Events::drogueDeploy_to_drogue_jerk(RocketData& rocket_data) {}
void MIDAS_Events::drogueDeploy_to_drouge_timed(RocketData& rocket_data) {}

// DROGUE
void MIDAS_Events::drogue_to_mainDeploy(RocketData& rocket_data) {}

// MAIN DEPLOY
void MIDAS_Events::mainDeploy_to_main_jerk(RocketData& rocket_data) {}
void MIDAS_Events::mainDeploy_to_main_timed(RocketData& rocket_data) {}

// MAIN
void MIDAS_Events::main_to_landed(RocketData& rocket_data) {}

// LANDED
void MIDAS_Events::landed_to_safe(RocketData& rocket_data) {
    rocket_data.command_flags.should_transition_pyro_test = false;
    rocket_data.command_flags.should_transition_idle = false;
    rocket_data.command_flags.should_transition_safe = false;
}
void MIDAS_Events::landed_to_main(RocketData& rocket_data) {}

#endif