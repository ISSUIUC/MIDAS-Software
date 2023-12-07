#pragma once

#include "kalman_filter.h"
#include "sensor_data.h"

class DisplacementKalmanFilter : public KalmanFilter<9, 4>
{
public:
    DisplacementKalmanFilter();

    void initialize() override;
    void initialize(Barometer& barometerData, Acceleration& imuData);
    void priori() override;
    void update() override;

    void setQ(float dt, float sd) override;
    void setF(float dt) override;

    KalmanData getState() override;
    void setState(KalmanData state) override;

    void kfTick(float dt, float sd, Barometer& barometerData, Acceleration& imuData);

private:
    // im just assuming 20hz rn
    float s_dt_ = 0.016f;
    float spectral_density_ = 13.0f;

};

extern DisplacementKalmanFilter displacement_kf;