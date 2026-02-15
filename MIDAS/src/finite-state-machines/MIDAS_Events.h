#include "rocket_state.h"

/**
 * @struct MIDAS_Events
 * 
 * @brief Abstracts changing command flags during state changes away from fsm.cpp.
 * The functions are defined in MIDAS_Events.cpp
 */
struct MIDAS_Events {
    void safe_to_pyroTest(RocketData& rocket_data);
    void safe_to_stateIdle(RocketData& rocket_data);
    
    void pyroTest_to_safe_forced(RocketData& rocket_data);
    void pyroTest_to_safe_timed(RocketData& rocket_data);
    void pyroTest_to_firstBoost(RocketData& rocket_data);

    void idle_to_safe_forced(RocketData& rocket_data);
    void idle_to_firstBoost(RocketData& rocket_data);

    void firstBoost_to_idle(RocketData& rocket_data);
    void firstBoost_to_burnout(RocketData& rocket_data);

    void burnout_to_firstBoost(RocketData& rocket_data);
    void burnout_to_sustainerIgnition(RocketData& rocket_data);
    void burnout_to_firstSeparation(RocketData& rocket_data);

    void sustainerIgnition_to_coast(RocketData& rocket_data);
    void sustainerIgnition_to_secondBoost(RocketData& rocket_data);

    void firstSeparation_to_coast_jerk(RocketData& rocket_data);
    void firstSeparation_to_coast_timed(RocketData& rocket_data);

    void secondBoost_to_sustainerIgnition(RocketData& rocket_data);
    void secondBoost_to_coast(RocketData& rocket_data);

    void coast_to_secondBoost(RocketData& rocket_data);
    void coast_to_apogee(RocketData& rocket_data);

    void apogee_to_coast(RocketData& rocket_data);
    void apogee_to_drogueDeploy(RocketData& rocket_data);

    void drogueDeploy_to_drogue_jerk(RocketData& rocket_data);
    void drogueDeploy_to_drouge_timed(RocketData& rocket_data);

    void drogue_to_mainDeploy(RocketData& rocket_data);

    void mainDeploy_to_main_jerk(RocketData& rocket_data);
    void mainDeploy_to_main_timed(RocketData& rocket_data);

    void main_to_landed(RocketData& rocket_data);

    void landed_to_safe(RocketData& rocket_data);
    void landed_to_main(RocketData& rocket_data);
};