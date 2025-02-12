#ifndef PID_H
#define PID_H

#include <vector>
using std::vector;
class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_, double Kp_Spd_, double Ki_Spd_, double Kd_Spd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte, double err_spd);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  vector<double> TotalError();
    
    double calc_tar(double tar_speed, double act_spd);
    double calc_traj(double tar_speed);
    vector<double> twiddle(double p[],double dp[], double cte, double err_spd);
    bool is_initialized;
    bool is_traj_initialized;
    double cte_mod;
    bool twiddle_avail;
    int n_iter;

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
    double p_error_spd;
    double i_error_spd;
    double d_error_spd;
    double total_err;
    double total_err_Spd;
    double cte_prev;
    double cte_acc;
    double err_spd_prev;
    double err_spd_acc;
    

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
    double Kp_Spd;
    double Ki_Spd;
    double Kd_Spd;
    double mod_speed;
    double mod_accel;
    int max_iter;
    double total_cte;
    int p_iterator;
    int fine_tune_count;
    bool first_change;
    bool second_change;
    double error_twiddle;
    double best_error;
    double best_p[3] = {Kp, Ki, Kd};
    double tol;
};

#endif  // PID_H
