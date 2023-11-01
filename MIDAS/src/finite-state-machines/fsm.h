#pragma once

//header for fsm governing state transitions and in-flight event control

#include "fsm_states.h"
#include "thresholds.h"
#include "sensor_data.h"
#include "FreeRTOSConfig.h"
#include "Buffer.h"
#include <numeric>

FSMState tick_fsm(FSMState curr_state, std::array<HighGData, 8> hg, std::array<Barometer, 8> bar);

double getAcceleration(std::array<HighGData, 8> hg) { //efficent enough?
    size_t size = hg.size();
    double avg = 0;

    for (size_t i = 0; i < size; i++) { avg += (hg.at(i).gz/(double)size); }
    
    return avg;
}

double getJerk(std::array<HighGData, 8> hg) {
    
    double avg1 = 0;
    for (size_t i = 0; i < hg.size()/2; i++)
    {
        avg1 += (hg.at(i).gz/hg.size());
    }

    double avg2 = 0;
    for (size_t i = hg.size()/2; i < hg.size(); i++)
    {
        avg2 += (hg.at(i).gz/hg.size());
    }
    
    double sec = tick_to_ms(hg.size()) * 1000;
    
    return (avg2 - avg1)/(sec);
}

double getVerticalSpeed(std::array<Barometer, 8> bar) {
    double avg1 = 0;
    for (size_t i = 0; i < bar.size()/2; i++)
    {
        avg1 += (bar.at(i).altitude/bar.size());
    }

    double avg2 = 0;
    for (size_t i = bar.size()/2; i < bar.size(); i++)
    {
        avg2 += (bar.at(i).altitude/bar.size());
    }
    
    double sec = tick_to_ms((TickType_t)bar.size()) * 1000;
    
    return (avg2 - avg1)/(sec);
}


double tick_to_ms(TickType_t ticks) { return ((double)ticks/configTICK_RATE_HZ)*1000;}
double launch_time;
double burnout_time;
double second_boost_time;
double coast_time;
double drogue_time;
double apogee_time;
double main_time;
double landed_time;
