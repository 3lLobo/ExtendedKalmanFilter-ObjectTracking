
#include <fstream>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "ground_truth_package.h"
#include "measurement_package.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {

  check_arguments(argc, argv);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  /**********************************************
   *  Set Measurements                          *
   **********************************************/

  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;

  string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long timestamp;

    // reads first element from the current line
    iss >> sensor_type;

    if (sensor_type.compare("L") == 0) {
      // laser 

      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float px;
      iss >> px;
      float py;
      iss >> py;
      meas_package.raw_measurements_ << px, py;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    } else if (sensor_type.compare("R") == 0) {
      // radar 

      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float phi;
      float rho_;
      float rho_dot_;
      iss >> rho_;
      iss >> phi;
      iss >> rho_dot_;
      meas_package.raw_measurements_ << rho_, phi, rho_dot_;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

      // read ground truth data
      float x_gt;
      iss >> x_gt;
      float y_gt;
      iss >> y_gt;
      float vx_gt;
      iss >> vx_gt;
      float vy_gt;
      iss >> vy_gt;
      gt_package.gt_values_ = VectorXd(4);
      gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
      gt_pack_list.push_back(gt_package);
  }
  // Create a Fusion EKF instance
  FusionEKF fusionEKF;

  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  size_t N = measurement_pack_list.size();
  for (size_t i = 0; i < N; ++i) {
    fusionEKF.ProcessMeasurement(measurement_pack_list[i]);
    out_file_ << fusionEKF.ekf_.x_(0) << "\t";
    out_file_ << fusionEKF.ekf_.x_(1) << "\t";
    out_file_ << fusionEKF.ekf_.x_(2) << "\t";
    out_file_ << fusionEKF.ekf_.x_(3) << "\t";

    // Measurements
    if (measurement_pack_list[i].sensor_type_ == MeasurementPackage::LASER) {
      // estimation in polar coordinates 
      out_file_ << measurement_pack_list[i].raw_measurements_(0) << "\t";
      out_file_ << measurement_pack_list[i].raw_measurements_(1) << "\t";
    } else if (measurement_pack_list[i].sensor_type_ == MeasurementPackage::RADAR) {
      // estimation in cartesian coordinates
      float rho_ = measurement_pack_list[i].raw_measurements_(0);
      float phi = measurement_pack_list[i].raw_measurements_(1);
      out_file_ << rho_ * cos(phi) << "\t"; // p1_meas
      out_file_ << rho_ * sin(phi) << "\t"; // ps_meas
    }

    out_file_ << gt_pack_list[i].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[i].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[i].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[i].gt_values_(3) << "\n";

    estimations.push_back(fusionEKF.ekf_.x_);
    ground_truth.push_back(gt_pack_list[i].gt_values_);
  }

  // print out RMSE
  Tools tools;
  cout << "The accuracy (RMSE) is:" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;
  cout << "Congrats!!! \n :)";
  // close all files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  return 0;
}
