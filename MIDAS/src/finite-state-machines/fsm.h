#pragma once

//header for fsm governing state transitions and in-flight event control

#include "fsm_states.h"
#include "thresholds.h"
#include "sensor_data.h"
#include "FreeRTOSConfig.h"
#include "Buffer.h"

FSMState tick_fsm(FSMState curr_state, HighGData hg);

double getAccelerationAverage() {
    return 
}

double tick_to_ms(TickType_t ticks) { return ((double)ticks/configTICK_RATE_HZ)*1000;}
double launch_time;
double burnout_time;
double second_boost_time;
double coast_time;
double drogue_time;

