#ifndef GNC_EXAMPLE_KF_H_
#define GNC_EXAMPLE_KF_H_

#include "kalman_filter.h"

// makes a kalman filter with 3 state variables and 3 sensor inputs
class ExampleKalmanFilter : public KalmanFilter<3, 3>
{
public:
    ExampleKalmanFilter();

    void initialize() override;
    void priori() override;
    void update() override;

    void setQ(float dt, float sd) override;
    void setF(float dt) override;

    KalmanData getState() override;
    void setState(KalmanData state) override;

private:
    KalmanData state;

};

extern ExampleKalmanFilter example_kf;

#endif  // GNC_EXAMPLE_KF_H_
