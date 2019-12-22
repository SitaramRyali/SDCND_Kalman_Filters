#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations,
	const vector<VectorXd>& ground_truth) {
	int est_len = estimations[0].size();
	VectorXd rmse(est_len);
	rmse = VectorXd::Zero(est_len);
	//rmse << 0, 0, 0, 0;
	// TODO: YOUR CODE HERE
	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size


	if (estimations.size() != ground_truth.size())
	{
		cout << "please input matrices with same shape!";
		return rmse;
	}
	else if (estimations.size() == 0)
	{
		cout << "Estimator matrix is with size of Zero!";
		return rmse;
	}

	// TODO: accumulate squared residuals
	for (int i = 0; i < estimations.size(); ++i) {
		// ... your code here
		VectorXd error = estimations[i] - ground_truth[i];
		VectorXd squared_error = error.array() * error.array();
		rmse = rmse + squared_error;
	}

	// TODO: calculate the mean
	rmse = rmse / estimations.size();
	// calculate the squared root
	rmse = rmse.array().sqrt();

	// return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	/**
	 * TODO:
	 * Calculate a Jacobian here.
	 */

	MatrixXd Hj(3, 4);
	// recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// TODO: YOUR CODE HERE 
	float denom = sqrt((px * px) + (py * py));
	float rr_numerator = (px * py) * (vx - vy);
	float rr_denom = pow(denom, 3);

	// check division by zero
	if (denom < 0.0001)
	{
		cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
	}
	else
	{
		Hj << (px / denom), (py / denom), 0, 0,
			((-py) / denom), (px / denom), 0, 0,
			(rr_numerator / rr_denom), ((-rr_numerator) / rr_denom), (px / denom), (py / denom);
	}
	// send the Jacobian matrix

	return Hj;

}
