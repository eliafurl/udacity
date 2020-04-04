#include "PID.h"
#include <numeric>
#include <iostream>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool twiddle) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;

  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;

  if (twiddle) {
    this->tp = {
      true,             //active
      0.05,             //tolerance
      0.0,              //best_error
      0,                //iteration
      {0.1, 0.1, 0.1}   //delta_p
    };
  }

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

  this->d_error = cte - this->p_error; // differential error
  this->p_error = cte;                // proportional error
  this->i_error += cte;                // integral error

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return 0.0;  // TODO: Add your total error calc here!
}

double PID::ComputeControl(){
  double control = - this->Kp * this->p_error - this->Kd * this->d_error - this->Ki * this->i_error;
  // steering actuator saturation
  if (control < this->MIN_CONTROL) {
    control = this->MIN_CONTROL;
  } else if (control > this->MAX_CONTROL) {
    control = this->MAX_CONTROL;
  }
  return control;
}

void PID::Twiddle(double error) {
  double sum_threshold = std::accumulate(this->tp.delta_p.begin(), this->tp.delta_p.end(), 0.0);
  if (true) {//(sum_threshold < this->tp.tolerance) {
    for (int i=0; i < this->gains.size(); i++) {
        //double dp = this->tp.delta_p[i];
        //*this->gains[i] += dp;
      if (error < this->tp.best_error) {
        this->tp.best_error = error;
        
      }
    }
  }


  std::cout << "[Twiddle] PID gains :\n"
            << "Kp: " << Kp << "\n"
            << "Kd: " << Kd << "\n"
            << "Ki: " << Ki << std::endl;
}