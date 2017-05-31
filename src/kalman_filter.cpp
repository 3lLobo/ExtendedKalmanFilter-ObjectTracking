#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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

void KalmanFilter::Predict() 
{

  x_ =F_ *x_;
  P_=F_ *P_ *F_.transpose() +Q_;
}
void KalmanFilter::Update(const VectorXd &z) 
{
	// update the state with KF equations
  VectorXd z_pred = H_ *x_;
  VectorXd y = z -z_pred;
  MatrixXd H_t = H_.transpose();
  MatrixXd S = H_ *P_ * H_t +R_;
  MatrixXd PHt = P_ *H_t;
  MatrixXd S_i = S.inverse();
  MatrixXd K = PHt * S_i;
	//now make a new estimate
  x_ = x_ + (K *y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I -K *H_) *P_;
  
}
void KalmanFilter::UpdateEKF(const VectorXd &z, const VectorXd &z_pred) {

  MatrixXd H_t = H_.transpose();  
  VectorXd y = atan2(sin(z -z_pred), cos(z -z_pred));
  MatrixXd S = H_ * P_ * H_t + R_;
  MatrixXd PHt = P_ * H_t;
  MatrixXd K = PHt *S.inverse();

  //now make a new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K *H_) *P_;
  
}
