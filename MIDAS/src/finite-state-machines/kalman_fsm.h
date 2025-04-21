#include "fsm_states.h" // fix import paths please

class KalmanFSM {
    public:
        KalmanFSM() = default;
        void tickFSM();
    private:
        FSMState rocket_state_ = FSMState::STATE_IDLE; // Initial state
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
};