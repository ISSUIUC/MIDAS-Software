#pragma once

class SAMMotor {
public:
    SAMMotor(float step_size, float actuation_time);

    void setSpeed(float speed);
    void update(float current_time);
    float getCurrentSpeed() const;
    float get_step_size() const;
    float get_gear_ratio() const;

private:
    float step_size_;
    float actuation_time_;
    float current_speed_;
    float target_speed_;
    float last_update_time_;
    float gear_ratio;
};
