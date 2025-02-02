#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController
{
private:
    double K_p_;
    double K_i_;
    double K_d_;
    double target_state_;
    double current_state_;
    double integral_term_ = 0.0;
    double prev_error_;
    bool has_prev_error_ = false;

public:
    PIDController(double K_p = 1.0, double K_i = 0.0, double K_d = 0.0, double target_state = 0.0);
    double get_control(double current_state, double dt = 0.0);
    double get_control_from_error(double error, double dt = 0.0);
    void set_target_state(double ts);
    double get_target_state();
};

#endif // PID_CONTROLLER_HPP