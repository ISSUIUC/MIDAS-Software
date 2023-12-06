git #include "example_kf.h"

ExampleKalmanFilter::ExampleKalmanFilter() : KalmanFilter() {}

void ExampleKalmanFilter::initialize() {}


void ExampleKalmanFilter::priori() {
    x_priori = (F_mat * x_k);
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}

// TODO: Finish this Methid
void ExampleKalmanFilter::update() {
    if (_curr_state == FSM_state::STATE_LAUNCH_DETECT) {
        float sum = 0;
        for (int i = 0; i < 8; i++) {
            sum+=_curr_baro_buf.pressure;
        }
        setState((KalmanData){sum / 10.0f, 0, 0, 0, 0, 0, 0, 0, 0});

void ExampleKalmanFilter::setQ(float dt, float sd) {}

void ExampleKalmanFilter::setF(float dt) {}

KalmanData ExampleKalmanFilter::getState() { return KalmanData(); }

void ExampleKalmanFilter::setState(KalmanData state)
{
    this->state = state;
}

ExampleKalmanFilter example_kf;
