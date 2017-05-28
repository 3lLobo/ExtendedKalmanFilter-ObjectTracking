#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:

  // Constructor
  Tools();

  // Destructor
  virtual ~Tools();

  // Helper method to calculate RMSE.
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  // Helper method to calculate Jacobians.
  MatrixXd CalculateJacobian(const VectorXd& x_state);

};

#endif /* TOOLS_H_ */
