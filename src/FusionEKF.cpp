#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;


  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1;

  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
	  const double rho = measurement_pack.raw_measurements_(0,0);
      const double phi = measurement_pack.raw_measurements_(1,0);
      const double rho_d = measurement_pack.raw_measurements_(2,0);
     
	  float x = rho * cos(phi);
      float y = rho * sin(phi);
      float vy = rho_d * sin(phi);
      float vx = rho_d * cos(phi);
      ekf_.x_ << x, y, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
	  
      */
		if (measurement_pack.raw_measurements_[0] == 0 or measurement_pack.raw_measurements_[1] == 0)
		{
          return;
        }
      ekf_.x_ << measurement_pack.raw_measurements_(0,0), measurement_pack.raw_measurements_(1,0), 0.0, 0.0;		
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

   
   // single time measurement
   
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  float noise_ax = 9;
  float noise_ay = 9;
  float dt_hoch2 = (dt * dt) / 2;
  MatrixXd G = MatrixXd(4,2);
  G << dt_hoch2, 0,
       0, dt_hoch2,
       dt, 0,
       0, dt ;
  MatrixXd QV = MatrixXd(2,2);
  QV << noise_ax, 0,
       0, noise_ay;
  ekf_.Q_ = G * QV * G.transpose();
  ekf_.Predict();




  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.H_ = Hj_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_, zpred);
    double phi = atan(ekf_.x_[1] / ekf_.x_[0]);
    double rho = sqrt(pow(ekf_.x_[0], 2) + pow(ekf_.x_[1], 2));
    double rho_d = ((ekf_.x_[0] * ekf_.x_[2] + ekf_.x_[1] * ekf_.x_[3]) / (sqrt(pow(ekf_.x_[0], 2) + pow(ekf_.x_[1], 2))));

    MatrixXd zpred(3, 1);
    zpred << rho, phi, rho_d;
	
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
	//Laser state and matrix update
    ekf_.Update(measurement_pack.raw_measurements_);	
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
