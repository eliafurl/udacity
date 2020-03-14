#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
   /**
   * Set the process and measurement noises
   */
  
  //measurement function
  H_laser_ << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;
  
  // measurement noises
  noise_ax_ = 9.0;
  noise_ay_ = 9.0;
  
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  
  VectorXd meas_data = VectorXd(measurement_pack.raw_measurements_.rows());
  meas_data << measurement_pack.raw_measurements_;
  cout << "data = " << meas_data << endl;
  
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     */

    // first measurement
    cout << "EKF: " << endl;
    auto x0 = VectorXd(4);
    x0 << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates 
      // and initialize state.
      float px = meas_data(0)*cos(meas_data(1));
      float py = meas_data(0)*sin(meas_data(1));
      float vx = 0;
      float vy = 0;
      x0 << px, py, vx, vy;

      previous_timestamp_ = measurement_pack.timestamp_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      float px = meas_data(0);
      float py = meas_data(1);
      x0 << px, py, 0.0, 0.0;

      previous_timestamp_ = measurement_pack.timestamp_;
    }
    else {
      // Error
      cout << "Sensor type Error - Not a supported sensor!" << endl;
      return;
    }

    auto P = MatrixXd(4, 4);
    auto F = MatrixXd(4, 4);
    auto Q = MatrixXd(4, 4);

    P << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1000, 0,
         0, 0, 0, 1000;

    F << 1.0, 0.0, 1.0, 0.0,
         0.0, 1.0, 0.0, 1.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;

    Q << 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0;

    ekf_.Init(x0, P, F, H_laser_, R_laser_, Q);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * Time is measured in seconds.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // Compute elapsed time
  auto d_timestamp = (measurement_pack.timestamp_ - previous_timestamp_)/ 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = d_timestamp * d_timestamp;
  float dt_3 = dt_2 * d_timestamp;
  float dt_4 = dt_3 * d_timestamp;

  // Update the state transition matrix F according to the new elapsed time.
  ekf_.F_(0, 2) = d_timestamp;
  ekf_.F_(1, 3) = d_timestamp;

  // Update the process noise covariance matrix.
  ekf_.Q_ <<  dt_4/4*noise_ax_, 0, dt_3/2*noise_ax_, 0,
              0, dt_4/4*noise_ay_, 0, dt_3/2*noise_ay_,
              dt_3/2*noise_ax_, 0, dt_2*noise_ax_, 0,
              0, dt_3/2*noise_ay_, 0, dt_2*noise_ay_;

  ekf_.Predict();
  /**
   * Update:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Linearize measurement function h(x) and set up Radar matrices

    Hj_ << tools.CalculateJacobian(ekf_.x_);
    cout << "Hj_ = " << Hj_ << endl; 
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    // Perform the measurement update
    ekf_.UpdateEKF(meas_data);

  } else {
    // Set up Laser matrices
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    // Perform the measurement update
    ekf_.Update(meas_data);
}
  
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
