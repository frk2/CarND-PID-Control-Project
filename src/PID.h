#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double dp;
  double di;
  double dd;

  double last_error;
  double total_error;
  double best_err;

  double last_err_window;
  int window_samples;
  int samples;
  int twiddling_index;
  int twiddle_check_mode;
  /*
   *
   *
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  double
  UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
