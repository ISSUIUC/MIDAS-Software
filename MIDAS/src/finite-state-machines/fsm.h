#pragma once

// header for fsm governing state transitions and in-flight event control

#include "FreeRTOSConfig.h"
#include "fsm_states.h"
#include "thresholds.h"
#include "sensor_data.h"
#include "Buffer.h"
#include <numeric>

class FSM {
   public:
    FSM() = default;

    FSMState tick_fsm_sustainer(FSMState curr_state, std::array<HighGData, 8> hg, std::array<Barometer, 8> bar);

    FSMState tick_fsm_booster(FSMState state, std::array<HighGData, 8> hg, std::array<Barometer, 8> bar);

    float getAcceleration(std::array<HighGData, 8> hg);

    float getAltitude(std::array<Barometer, 8> bar);

    double getJerk(std::array<HighGData, 8> hg);

    double getVerticalSpeed(std::array<Barometer, 8> bar);

    double tick_to_ms(unsigned int ticks) { return ((double)ticks / configTICK_RATE_HZ) * 1000; }

   private:
    double launch_time;
    double burnout_time;
    double sustainer_ignition_time;
    double second_boost_time;
    double coast_time;
    double drogue_time;
    double apogee_time;
    double main_time;
    double landed_time;
    double first_seperation_time;

};
