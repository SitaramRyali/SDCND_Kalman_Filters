#include <iostream>
#include <vector>
#include "Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {
	/**
	 * Compute the Jacobian Matrix
	 */

	 // predicted state example
	 // px = 1, py = 2, vx = 0.2, vy = 0.4
	VectorXd x_predicted(4);
	x_predicted << 1, 2, 0.2, 0.4;

	MatrixXd Hj = CalculateJacobian(x_predicted);

	cout << "Hj:" << endl << Hj << endl;

	return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

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
	if (denom == 0)
	{
		
	}
	else
	{
		Hj << (px / denom), (py / denom), 0, 0,
		((-py) / denom), (px / denom), 0, 0,
		(rr_numerator / rr_denom), ((-rr_numerator) / rr_denom), (px / denom), (py / denom);
	}
	// compute the Jacobian matrix

	return Hj;
}