#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <iostream>
#include "Eigen/Dense"
#include <vector>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:

  // State vector
  VectorXd x_;

  // State covariance matrix
  MatrixXd P_;

  // State transistion matrix
  MatrixXd F_;

  // Process covariance matrix
  MatrixXd Q_;

  // Measurement matrix
  MatrixXd H_;

  // Measurement covariance matrix
  MatrixXd R_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
      MatrixXd &Q_in, MatrixXd &H_in, MatrixXd &R_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const VectorXd &z, const VectorXd &z_pred);

};

#endif /* KALMAN_FILTER_H_ */
