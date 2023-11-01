#pragma once

//header for fsm governing state transitions and in-flight event control

#include "fsm_states.h"
#include "thresholds.h"
#include "sensor_data.h"
#include "FreeRTOSConfig.h"
#include "Buffer.h"
#include <numeric>

FSMState tick_fsm(FSMState curr_state, std::array<HighGData, 8> hg, std::array<Barometer, 8> bar);

double getAcceleration(std::array<HighGData, 8> hg) {
    
}
template<typename T>
double getAverage(std::array<T, 8> array) {


}
double getJerk(std::array<HighGData, 8> hg) {
    
    return;
}

double getVerticalSpeed(std::array<Barometer, 8> bar) {
    
    return;
}


double tick_to_ms(TickType_t ticks) { return ((double)ticks/configTICK_RATE_HZ)*1000;}
double launch_time;
double burnout_time;
double second_boost_time;
double coast_time;
double drogue_time;
double apogee_time;
double main_time;
