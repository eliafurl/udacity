#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * Predict the state
   */
  x_ << F_*x_;
  P_ << F_*P_*F_.transpose() + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * Update the state by using Kalman Filter equations
   */
  auto y = VectorXd(z.rows());
  auto S = MatrixXd(R_.rows(), R_.cols());
  auto K = MatrixXd(P_.rows(), S.cols());
  auto I = MatrixXd::Identity(x_.rows(), x_.rows());
 
  
  y << z - H_*x_;
  S << H_*P_*H_.transpose() + R_;
  K << P_*H_.transpose()*S.inverse();
  
  x_ << x_ + K*y;
  P_ << (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * updates the state by using Extended Kalman Filter equations
   */
  auto y = VectorXd(z.rows());
  auto S = MatrixXd(R_.rows(), R_.cols());
  auto K = MatrixXd(P_.rows(), S.cols());
  auto I = MatrixXd::Identity(x_.rows(), x_.rows());
  
  y << z - MeasurementFunction_Radar();
  // wrap angle phi in [-pi,pi]
  y(1) = atan2(sin(y(1)), cos(y(1)));

  S << H_*P_*H_.transpose() + R_;
  K << P_*H_.transpose()*S.inverse();
  
  x_ << x_ + K*y;
  P_ << (I - K*H_)*P_;
}

VectorXd KalmanFilter::MeasurementFunction_Radar() {

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(pow(px,2) + pow(py,2));
  VectorXd h_x = VectorXd(3);

  h_x << rho, atan2(py, px), (px*vx + py*vy)/rho;
  return h_x;
}