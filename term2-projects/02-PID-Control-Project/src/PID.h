#ifndef PID_H
#define PID_H
#include <vector>
struct TwiddleParameters
{
  bool active;
  double tolerance;
  double best_error;
  int iteration;
  std::vector<double> delta_p;
};

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
  void Init(double Kp_, double Ki_, double Kd_, bool twiddle);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * @brief Compute the control action
   * @output the steering control
   */
  double ComputeControl();

  /**
   * @brief Twiddle algorithm for PID coefficients tuning
   *
   */
  void Twiddle(double error);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */
  double Kp;
  double Ki;
  double Kd;
  std::vector<double*> gains = {&Kp, &Ki, &Kd};

  /**
   * Steering actuator saturation
   */
  const double MIN_CONTROL = -1.0;
  const double MAX_CONTROL = 1.0;
  /**
   * Twiddle parameters
   */
  TwiddleParameters tp;
};

#endif  // PID_H