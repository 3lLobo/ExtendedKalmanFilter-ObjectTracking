#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0; 

	for(int i=0; i < estimations.size(); ++i){
		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array()*residual.array();
		rmse += residual;
	}		
	// Mean of rmse and squareroot
  	rmse = rmse/estimations.size();
	rmse = sqrt(rmse.array());
	
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	// Jacobian
	MatrixXd Hj(3, 4);

	// Pull parameters out of state
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// Start pre-compute of repeating denominators
	float x2y2 = px*px + py*py;      // x^2+y^2

	// Avoid Division by zero, set px*px + py*py to small value
	if (x2y2 < 0.0001) {
		x2y2 = 0.0001;
	}

	// Finish pre-compute
	float x2y2_a = sqrt(x2y2);      // (x^2+y^2)^0.5
	float x2y2_b = x2y2 * x2y2_a;  // (x^+y^2)^1.5

	// Compute the Jacobian matrix
	Hj << (px / x2y2_a), (py / x2y2_a), 0, 0,
		-(py / x2y2), (px / x2y2), 0, 0,
		(py*(vx*py - vy*px) / x2y2_b), (px*(px*vy - py*vx) / x2y2_b), (px / x2y2_a), (py / x2y2_a);

	return Hj;
}

