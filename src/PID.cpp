#include "PID.h"
#include <vector>
#include <iostream>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */
using namespace std;
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
    is_traj_initialized = false;
    twiddle_avail = true;
    first_change = true;
    second_change = true;
    cte_mod = 0.0;
    n_iter = 0;
    max_iter = 4;
    total_cte = 0.0;
    error_twiddle = 0.0;
    p_iterator = 0;
    best_error = 10000.00;
    fine_tune_count = 0;
    tol = 0.0001;
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
    double kp = 0.05;
    double tar_spd_ = tar_speed / 2.24;
    double act_spd_ = act_spd / 2.24;
    
    mod_spd = act_spd_ + fmax(fmin(kp * (tar_spd_ - act_spd_), 0.5), -0.5);
    return mod_spd * 2.24;
}

double PID::calc_traj(double tar_speed){
    if (!is_traj_initialized){
        mod_speed = 0.0;
        mod_accel = 0.0;
        is_traj_initialized = true;
    }
    double kp_speed = 0.05;
    double kp_accel = 0.05;
    double tar_spd_ = tar_speed / 2.24;
    
    double tar_accel_raw = fmax(fmin(kp_speed * (tar_spd_ - mod_speed), 0.1), -0.1);
    double tar_jerk_raw = fmax(fmin(kp_accel * (tar_accel_raw - mod_accel), 0.05), -0.05);
    //mod_speed = act_spd_ + fmax(fmin(kp * (tar_spd_ - act_spd_), 1.0), -1.0);
    //intergrate the components:
    mod_accel = mod_accel + tar_jerk_raw;
    mod_speed = mod_speed + mod_accel;
    return mod_speed * 2.24;
}

vector<double> PID::twiddle(double p[],double dp[], double cte, double err_spd)
{
    vector<double> control_output;
    if (twiddle_avail == true)
    {
        total_cte += pow(cte,2);
        UpdateError(cte, err_spd);
        control_output = TotalError();
        n_iter++;
        if (n_iter > max_iter)
        {
            if (first_change == true)
            {
                p[p_iterator] += dp[p_iterator];
                std::cout << "step change: p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << endl;
                first_change = false;
            }
            else
            {
                error_twiddle = total_cte / max_iter;
                if (error_twiddle < best_error && second_change == true)
                {
                    best_error = error_twiddle;
                    best_p[0] = p[0];
                    best_p[1] = p[1];
                    best_p[2] = p[2];
                    dp[p_iterator] *= 1.5;
                    fine_tune_count += 1;
                }
                else
                {
                    if (second_change == true)
                    {
                        p[p_iterator] -= 2 * dp[p_iterator];
                        std::cout << "correction: p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << endl;
                        second_change = false;
                    }
                    else
                    {
                        if (error_twiddle < best_error)
                        {
                            best_error = error_twiddle;
                            best_p[0] = p[0];
                            best_p[1] = p[1];
                            best_p[2] = p[2];
                            dp[p_iterator] *= 1.5;
                            fine_tune_count += 1;
                        }
                        else
                        {
                            p[p_iterator] += dp[p_iterator];
                            dp[p_iterator] *= 0.5;
                            fine_tune_count += 1;
                        }
                        std::cout << "fine tune2: p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << endl;
                    }
                }
            }
            if(fine_tune_count > 0)
            {
              p_iterator = p_iterator+1;
              first_change = true;
              second_change = true;
              fine_tune_count = 0;
            }
            if(p_iterator == 3)
            {
              p_iterator = 0;
            }
            total_cte = 0.0;
            n_iter = 0;
            double sum_dp = dp[0] + dp[1] + dp[2];
            std::cout << " sumdp " << sum_dp << endl;
            if (sum_dp < tol)
            {
                twiddle_avail = false;
            }
        }
    }
    else
    {
        UpdateError(cte, err_spd);
        control_output = TotalError();
    }
    return control_output;
}
