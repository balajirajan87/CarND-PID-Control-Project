#ifndef PID_H
#define PID_H

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
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte, double str_val);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
    
    bool is_initialized;

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
    double total_err;
    double cte_prev;
    double cte_acc;
    double str_prev;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
};

#endif  // PID_H
