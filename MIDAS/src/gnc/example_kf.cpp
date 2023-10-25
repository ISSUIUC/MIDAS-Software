#include "example_kf.h"

ExampleKalmanFilter::ExampleKalmanFilter() : KalmanFilter() {}

void ExampleKalmanFilter::initialize() {
    

}

void ExampleKalmanFilter::priori() {}

void ExampleKalmanFilter::update() {}

void ExampleKalmanFilter::setQ(float dt, float sd) {
    Q(0, 0) = pow(dt, 5) / 20;
    Q(0, 1) = pow(dt, 4) / 8;
    Q(0, 2) = pow(dt, 3) / 6;
    Q(1, 1) = pow(dt, 3) / 8;
    Q(1, 2) = pow(dt, 2) / 2;
    Q(2, 2) = dt;
    Q(1, 0) = Q(0, 1);
    Q(2, 0) = Q(0, 2);
    Q(2, 1) = Q(1, 2);

    Q(3, 3) = pow(dt, 5) / 20;
    Q(3, 4) = pow(dt, 4) / 8;
    Q(3, 5) = pow(dt, 3) / 6;
    Q(4, 4) = pow(dt, 3) / 8;
    Q(4, 5) = pow(dt, 2) / 2;
    Q(5, 5) = dt;
    Q(4, 3) = Q(3, 4);
    Q(5, 3) = Q(3, 5);
    Q(5, 4) = Q(4, 5);

    Q(6, 6) = pow(dt, 5) / 20;
    Q(6, 7) = pow(dt, 4) / 8;
    Q(6, 8) = pow(dt, 3) / 6;
    Q(7, 7) = pow(dt, 3) / 8;
    Q(7, 8) = pow(dt, 2) / 2;
    Q(8, 8) = dt;
    Q(7, 6) = Q(6, 7);
    Q(8, 6) = Q(6, 8);
    Q(8, 7) = Q(7, 8);

    Q *= sd;
}

void ExampleKalmanFilter::setF(float dt) {
        for (int i = 0; i < 3; i++) {
        F_mat(3 * i, 3 * i + 1) = dt;
        F_mat(3 * i, 3 * i + 2) = (dt * dt) / 2;
        F_mat(3 * i + 1, 3 * i + 2) = dt;

        F_mat(3 * i, 3 * i) = 1;
        F_mat(3 * i + 1, 3 * i + 1) = 1;
        F_mat(3 * i + 2, 3 * i + 2) = 1;
    }
}

KalmanData ExampleKalmanFilter::getState() { return state; }

void ExampleKalmanFilter::setState(KalmanData state) { this->state = state; }

void ExampleKalmanFilter::kfTickFunction(FSM_state& curr_state, float dt) {
    if (curr_state >= FSM_state::STATE_IDLE) {
        setF(dt / 1000.0);
        setQ(dt / 1000.0, spectral_density);
        priori();
        update();
    }
}

ExampleKalmanFilter example_kf;
