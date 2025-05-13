#include "motor.h"

SAMMotor::SAMMotor(float step_size, float actuation_time) 
    : step_size_(step_size),
      actuation_time_(actuation_time),
      current_speed_(0.0f),
      target_speed_(0.0f),
      gear_ratio(1.0f) // need the real gear ratio
{
}

// Set the target speed of the motor
void SAMMotor::setSpeed(float speed) {
    target_speed_ = speed;
}

// Update the motor's speed based on the target speed and time elapsed
void SAMMotor::update(float current_time) {
    float dt = current_time - last_update_time_;
    if (dt > actuation_time_) {
        current_speed_ = target_speed_;
    } else {
        current_speed_ += (target_speed_ - current_speed_) * (dt / actuation_time_);
    }
    last_update_time_ = current_time;
}

// Get the current speed of the motor
float SAMMotor::getCurrentSpeed() const {
    return current_speed_;
}

// Get the step size of the motor
float SAMMotor::get_step_size() const {
    return step_size_;
}

// Get the gear ratio of the motor
float SAMMotor::get_gear_ratio() const {
    return 1.0f; // need the real gear ratio
}
