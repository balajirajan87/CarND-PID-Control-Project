#include "PID.h"
#include <vector>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, double Kp_Spd_, double Ki_Spd_, double Kd_Spd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    Kp_Spd = Kp_Spd_;
    Ki_Spd = Ki_Spd_;
    Kd_Spd = Kd_Spd_;
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    p_error_spd = 0.0;
    i_error_spd = 0.0;
    d_error_spd = 0.0;
    err_spd_prev = 0.0;
    err_spd_acc = 0.0;
    cte_prev = 0.0;
    cte_acc = 0.0;
    total_err = 0.0;
    is_initialized = true;
}

void PID::UpdateError(double cte, double err_spd) {
  /**
   * TODO: Update PID errors based on cte.
   */
    p_error = Kp * cte;
    p_error_spd = Kp_Spd * err_spd;
    double d_cte = cte - cte_prev;
    double d_err_spd = err_spd - err_spd_prev;
    //double d_str = str_val - str_prev;
    d_error = Kd * d_cte;
    d_error_spd = Kd_Spd * d_err_spd;
    cte_acc += cte;
    err_spd_acc += err_spd;
    i_error = Ki * cte_acc;
    i_error_spd = Ki_Spd * err_spd_acc;
    cte_prev = cte;
    err_spd_prev = err_spd;
    
}

vector<double> PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
    vector<double> total_error;
    total_err = -p_error - i_error - d_error;
    total_err_Spd = p_error_spd + i_error_spd + d_error_spd;
    total_error.push_back(total_err);
    total_error.push_back(total_err_Spd);
    return total_error;  // TODO: Add your total error calc here!
}

double PID::calc_tar(double tar_speed, double act_spd){
    double mod_spd;
    double kp = 0.1;
    double tar_spd_ = tar_speed / 2.24;
    double act_spd_ = act_spd / 2.24;
    
    mod_spd = act_spd_ + fmax(fmin(kp * (tar_spd_ - act_spd_), 1.0), -1.0);
    return mod_spd * 2.24;
}
