#include "rocket_state.h"

/**
 * @struct MIDAS_Events
 * 
 * @brief Abstracts changing command flags during state changes away from fsm.cpp.
 * The functions are defined in MIDAS_Events.cpp
 */
struct MIDAS_Events {
    void safe_to_pyroTest(CommandFlags& commands);
    void safe_to_stateIdle(CommandFlags& commands);
    
    void pyroTest_to_safe_forced(CommandFlags& commands);
    void pyroTest_to_safe_timed(CommandFlags& commands);
    void pyroTest_to_firstBoost(CommandFlags& commands);

    void idle_to_safe_forced(CommandFlags& commands);
    void idle_to_firstBoost(CommandFlags& commands);

    void firstBoost_to_idle(CommandFlags& commands);
    void firstBoost_to_burnout(CommandFlags& commands);

    void burnout_to_firstBoost(CommandFlags& commands);
    void burnout_to_sustainerIgnition(CommandFlags& commands);
    void burnout_to_firstSeparation(CommandFlags& commands);

    void sustainerIgnition_to_coast(CommandFlags& commands);
    void sustainerIgnition_to_secondBoost(CommandFlags& commands);

    void firstSeparation_to_coast_jerk(CommandFlags& commands);
    void firstSeparation_to_coast_timed(CommandFlags& commands);

    void secondBoost_to_sustainerIgnition(CommandFlags& commands);
    void secondBoost_to_coast(CommandFlags& commands);

    void coast_to_secondBoost(CommandFlags& commands);
    void coast_to_apogee(CommandFlags& commands);

    void apogee_to_coast(CommandFlags& commands);
    void apogee_to_drogueDeploy(CommandFlags& commands);

    void drogueDeploy_to_drogue_jerk(CommandFlags& commands);
    void drogueDeploy_to_drouge_timed(CommandFlags& commands);

    void drogue_to_mainDeploy(CommandFlags& commands);

    void mainDeploy_to_main_jerk(CommandFlags& commands);
    void mainDeploy_to_main_timed(CommandFlags& commands);

    void main_to_landed(CommandFlags& commands);

    void landed_to_safe(CommandFlags& commands);
    void landed_to_main(CommandFlags& commands);
};