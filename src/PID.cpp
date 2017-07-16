#include "PID.h"
#include "math.h"
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  dp = .2;
  dd = .5;
  di = 0.0005;
  samples = 0;
  last_err_window = 0;
  last_error = 0;
  total_error = 0;
  twiddling_index = 0;
  twiddle_check_mode = -1;
  window_samples = 0;
}

double PID::UpdateError(double cte) {
  if (last_error == 0) {
    last_error = cte;
  }

  if (twiddle_check_mode == -1) {
    best_err = last_err_window;
  }
  samples++;
  window_samples++;
  double error_diff = cte - last_error;
  last_error = cte;
  last_err_window += cte;
  total_error += cte;
  double steer = - Kp * cte - Kd * error_diff - Ki * total_error;
  steer = fmax(-1.0, steer);
  steer = fmin(1.0, steer);
  if (samples > 100 && window_samples > 100) {
    std::cout << "dd: " << dd << " dp: " << dp << " di:" << di <<std::endl;
    std::cout << "kd: " << Kd << " kp: " << Kp << " ki:" << Ki <<std::endl;

    window_samples = 0;
    if (twiddle_check_mode == -1) {
      twiddle_check_mode = 0;
      best_err = last_err_window;
    }

    if (twiddle_check_mode == 0) {
      if (twiddling_index == 0) {
        Kp += dp;
      } else if (twiddling_index == 1) {
        Kd += dd;
      } else if (twiddling_index == 2) {
        Ki += di;
      }
      twiddle_check_mode = 1;
    } else if (twiddle_check_mode == 1) {
      if (twiddling_index == 0) {
        if (fabs(last_err_window) < fabs(best_err)) {
          dp *= 1.1;
          best_err = last_err_window;
          twiddling_index = 1;
          twiddle_check_mode = 0;
        } else {
          Kp -= 2 * dp;
          twiddle_check_mode = 2;
        }
      } else if (twiddling_index == 1) {
        if (fabs(last_err_window) < fabs(best_err)) {
          dd *= 1.1;
          best_err = last_err_window;
          twiddling_index = 2;
          twiddle_check_mode = 0;
        } else {
          Kd -= 2 * dd;
          twiddle_check_mode = 2;
        }
      } else if (twiddling_index == 2) {
        if (fabs(last_err_window) < fabs(best_err)) {
          di *= 1.1;
          best_err = last_err_window;
          twiddling_index = 0;
          twiddle_check_mode = 0;
        } else {
          Ki -= 2 * di;
          twiddle_check_mode = 2;
        }
      }
    } else if (twiddle_check_mode == 2) {
      if (twiddling_index == 0) {
        if (fabs(last_err_window) < fabs(best_err)) {
          best_err = last_err_window;
          dp *= 1.1;
        } else {
          Kp += dp;
          dp *= 0.9;
        }
        twiddling_index = 1;
        twiddle_check_mode = 0;
      } else if (twiddling_index == 1) {
        if (fabs(last_err_window) < fabs(best_err)) {
          best_err = last_err_window;
          dd *= 1.1;
        } else {
          Kd += dd;
          dd *= 0.9;
        }
        twiddling_index = 2;
        twiddle_check_mode = 0;
      } else if (twiddling_index == 2) {
        if (fabs(last_err_window) < fabs(best_err)) {
          best_err = last_err_window;
          di *= 1.1;
        } else {
          Ki += di;
          di *= 0.9;
        }
        twiddling_index = 0;
        twiddle_check_mode = 0;
      }
    }
    last_err_window = 0;
  }

  return steer;
}

double PID::TotalError() {
  return total_error;
}

