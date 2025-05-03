#pragma once
#include "fsm_states.h"
#include "fsm.h"
class KalmanFSM : public FSM  {
   public:
    KalmanFSM() = default;
    void tick_fsm();
    FSMState get_rocket_state_fsm();
    private:
    FSMState rocket_state_ = FSMState::STATE_IDLE;
    double launch_time_ = 0;
    double burnout_time_ = 0;
    double sustainer_ignition_time_ = 0;
    double second_boost_time_ = 0;
    double coast_time_ = 0;
    double drogue_time_ = 0;
    double apogee_time_ = 0;
    double main_time_ = 0;
    double main_deployed_time_ = 0;
    double landed_time_ = 0;
    double pyro_test_entry_time_ = 0;
    double getAltitudeAverage(size_t start, size_t len);
    double getSecondDerivativeAltitudeAverage(size_t start, size_t len);
    double getAccelerationAverage(size_t start, size_t len);
};