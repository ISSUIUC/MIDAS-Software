#pragma once

#include "kalman_filter.h"
#include "sensor_data.h"

class Yessir : public KalmanFilter<3, 3>
{
public:
    Yessir();

    void initialize() override;
    void priori() override;
    void update() override;

    void setQ(float dt, float sd);
    void setF(float dt);

    KalmanData getState() override;
    void setState(KalmanData state) override;

    void tick(float dt, float sd, Barometer &barometerData, Acceleration &imuData);
    void reset();
private:
    float s_dt_ = 0.05f;
    float spectral_density_ = 13.0f;

    KalmanData state;
};

extern Yessir yessir;
