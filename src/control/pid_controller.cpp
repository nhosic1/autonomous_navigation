#include "autonomous_navigation/control/pid_controller.hpp"

PIDController::PIDController(double K_p, double K_i, double K_d, double target_state): K_p_(K_p), K_i_(K_i), K_d_(K_d), target_state_(target_state) {}

double PIDController::get_control(double current_state, double dt)
{
    double error = target_state_ - current_state;
    return get_control_from_error(error, dt);
}

double PIDController::get_control_from_error(double error, double dt)
{
    integral_term_ += error * dt;

    double derivative_term;
    if (has_prev_error_ && dt > 0.0)
        derivative_term = (error - prev_error_) / dt;
    else
    {
        derivative_term = 0.0;
        has_prev_error_ = true;
    }

    prev_error_ = error;

    double control = K_p_ * error + K_i_ * integral_term_ + K_d_ * derivative_term;
    return control;
}

void PIDController::set_target_state(double ts)
{
    target_state_ = ts;
    integral_term_ = 0.0;
    has_prev_error_ = false;
}

double PIDController::get_target_state()
{
    return target_state_;
}
