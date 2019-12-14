#include "tracking.h"
#include <iostream>
#include "Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;
using std::pow;

Tracking::Tracking() {
	is_initialized_ = false;
	previous_timestamp_ = 0;

	// create a 4D state vector, we don't know yet the values of the x state
	kf_.x_ = VectorXd(4);

	// state covariance matrix P
	kf_.P_ = MatrixXd(4, 4);
	kf_.P_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;


	// measurement covariance
	kf_.R_ = MatrixXd(2, 2);
	kf_.R_ << 0.0225, 0,
		0, 0.0225;

	// measurement matrix
	kf_.H_ = MatrixXd(2, 4);
	kf_.H_ << 1, 0, 0, 0,
		0, 1, 0, 0;

	// the initial transition matrix F_
	kf_.F_ = MatrixXd(4, 4);
	kf_.F_ << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;

	// set the acceleration noise components
	noise_ax = 5;
	noise_ay = 5;
}

Tracking::~Tracking() {

}

// Process a single measurement
void Tracking::ProcessMeasurement(const MeasurementPackage& measurement_pack) {
	if (!is_initialized_) {
		//cout << "Kalman Filter Initialization " << endl;

		// set the state with the initial location and zero velocity
		kf_.x_ << measurement_pack.raw_measurements_[0],
			measurement_pack.raw_measurements_[1],
			0,
			0;

		previous_timestamp_ = measurement_pack.timestamp_;
		is_initialized_ = true;
		return;
	}

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
	kf_.F_(0, 2) = dt;
	kf_.F_(1, 3) = dt;

	float var_x4_ = pow(dt, 4) * noise_ax * 0.25;
	float var_x3_ = pow(dt, 3) * noise_ax * 0.5;
	float var_x2_ = pow(dt, 2) * noise_ax;
	float var_y4_ = pow(dt, 4) * noise_ay * 0.25;
	float var_y3_ = pow(dt, 3) * noise_ay * 0.5;
	float var_y2_ = pow(dt, 2) * noise_ay;
	kf_.Q_ = MatrixXd(4, 4);
	kf_.Q_ << var_x4_, 0, var_x3_, 0,
		0, var_y4_, 0, var_y3_,
		var_x3_, 0, var_x2_, 0,
		0, var_y3_, 0, var_y2_;
	kf_.Predict();
	kf_.Update(measurement_pack.raw_measurements_);

	cout << "x_= " << kf_.x_ << endl;
	cout << "P_= " << kf_.P_ << endl;
}