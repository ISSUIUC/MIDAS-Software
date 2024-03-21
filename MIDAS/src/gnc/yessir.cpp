#include "yessir.h"

Yessir::Yessir() : KalmanFilter() {
    state = KalmanData();
}

void Yessir::initialize() {}

void Yessir::priori() {}

void Yessir::update() {}

void Yessir::tick(float dt, float sd, Barometer &barometerData, Acceleration &imuData) {
    // double integrate accelerometer data and store in state struct
    state.acceleration = {
        .ax = imuData.ax * 9.8f,
        .ay = imuData.ay * 9.8f,
        .az = imuData.az * 9.8f
    };

    state.velocity = {
        .vx = state.velocity.vx + state.acceleration.ax * dt,
        .vy = state.velocity.vy + state.acceleration.ay * dt,
        .vz = state.velocity.vz + state.acceleration.az * dt
    };

    state.position = {
        .px = state.position.px + state.velocity.vx * dt + 0.5f * state.acceleration.ax * dt * dt,
        .py = state.position.py + state.velocity.vy * dt + 0.5f * state.acceleration.ay * dt * dt,
        .pz = state.position.pz + state.velocity.vz * dt + 0.5f * state.acceleration.az * dt * dt
    };
    
}

KalmanData Yessir::getState() {
    return state;
}

void Yessir::setState(KalmanData state) {
    this->state = state;
}

void Yessir::setQ(float dt, float sd) {}

void Yessir::setF(float dt) {}

Yessir yessir;
