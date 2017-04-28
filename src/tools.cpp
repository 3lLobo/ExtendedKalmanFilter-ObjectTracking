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
  // now calculate the RMSE
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  
  if(estimations.size() != ground_truth.size() || estimations.size() == 0){
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  for(unsigned int i =0; i< estimations.size(); i++){
    VectorXd residual = estimations[i] - ground_truth[i];

    residual = residual.array() * residual.array();
    rmse += residual;
  }
	//calculate the mean
  rmse = rmse / estimations.size();
  //take the square root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	//define Jacobian Matrix
	MatrixXd Hj(3,4);
	
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// more often used terms
	float t_1 = px * px + py * py;
	float t_2 = sqrt(t_1);
	float t_3 = (t_1 * t_2);

	// error if devided by zero
	if(fabs(t_1) < 0.0001){
		cout << "Error! /n Division by Zero while calculating Jacobian matrix" << endl;
		return Hj;
	}

	// calculate Jacobian matrix
	Hj << (px / t_2), (py / t_2), 0, 0, -(py / t_1), (px / t_1), 0, 0, py * (vx * py - vy * px) / t_3, px * (px * vy - py * vx) / t_3, px / t_2, py / t_2;

	return Hj;
  
}
