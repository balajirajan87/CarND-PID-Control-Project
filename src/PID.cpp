#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    cte_prev = 0.0;
    str_prev = 0.0;
    cte_acc = 0.0;
    total_err = 0.0;
    is_initialized = true;
}

void PID::UpdateError(double cte, double str_val) {
  /**
   * TODO: Update PID errors based on cte.
   */
    p_error = Kp * cte;
    double d_cte = cte - cte_prev;
    //double d_str = str_val - str_prev;
    d_error = Kd * d_cte;
    cte_acc += cte;
    i_error = Ki * cte_acc;
    cte_prev = cte;
    str_prev = str_val;
    
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
    total_err = -p_error - i_error - d_error;
    return total_err;  // TODO: Add your total error calc here!
}
