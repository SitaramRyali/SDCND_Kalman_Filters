#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

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
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0,
	  0, 1, 0, 0;

  Hj_ << 1, 0, 0, 0,
	  0, 1, 0, 0,
	  0, 0, 1, 0,
	  0, 0, 0, 1;


  // the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
	  0, 1, 0, 1,
	  0, 0, 1, 0,
	  0, 0, 0, 1;







}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
	 // ekf_.x_ = measurement_pack.raw_measurements_;
	 // state covariance matrix P
	  ekf_.P_ = MatrixXd(4, 4);
	  ekf_.P_ << 1, 0, 0, 0,
		  0, 1, 0, 0,
		  0, 0, 1000, 0,
		  0, 0, 0, 1000;

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.

		float meas_rho = measurement_pack.raw_measurements_(0);
		float meas_phi = measurement_pack.raw_measurements_(1);
		float meas_rho_dot = measurement_pack.raw_measurements_(2);
		/*To convert from Polar Coordinates(r,phi) to Cartesian Coordinates(x, y) :
			x = r * cos(phi)
			y = r * sin(phi)*/

		ekf_.x_(0) = meas_rho * cos(meas_phi);
		ekf_.x_(1) = meas_rho * sin(meas_phi);
		ekf_.x_(2) = meas_rho_dot * cos(meas_phi);
		ekf_.x_(3) = meas_rho_dot * sin(meas_phi);

		//time_stamp = measurement_pack.raw_measurements_(3);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
		ekf_.x_(0) = measurement_pack.raw_measurements_(0);
		ekf_.x_(1) = measurement_pack.raw_measurements_(1);

		//time_stamp = measurement_pack.raw_measurements_(2);
    }

	previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float noise_ax = 9;
  float noise_ay = 9;

   // compute the time elapsed between the current and previous measurements
   // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;


  // TODO: YOUR CODE HERE
	// 1. Modify the F matrix so that the time is integrated
	// 2. Set the process covariance matrix Q
	// 3. Call the Kalman Filter predict() function
	// 4. Call the Kalman Filter update() function
	//      with the most recent raw measurements_
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  float var_x4_ = pow(dt, 4) * noise_ax * 0.25;
  float var_x3_ = pow(dt, 3) * noise_ax * 0.5;
  float var_x2_ = pow(dt, 2) * noise_ax;
  float var_y4_ = pow(dt, 4) * noise_ay * 0.25;
  float var_y3_ = pow(dt, 3) * noise_ay * 0.5;
  float var_y2_ = pow(dt, 2) * noise_ay;
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << var_x4_, 0, var_x3_, 0,
	  0, var_y4_, 0, var_y3_,
	  var_x3_, 0, var_x2_, 0,
	  0, var_y3_, 0, var_y2_;
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  Tools tools;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
	  ekf_.H_ = tools.CalculateJacobian(measurement_pack.raw_measurements_);
	  ekf_.Update(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
	  ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
