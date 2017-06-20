#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double tot_cte;
  double prev_cte;
  double err;
  double best_err;
  double tolerance;
  double roll_avg;
  int restart_sim;
  int span;
  bool tune_param;
  std::vector<double> K_vec;
  std::vector<double> d_p;
  int opt_track;
  int index;
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;


  double steer;
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, bool tune_param);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
