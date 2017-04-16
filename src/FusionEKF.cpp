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
  
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  
  //create a 4D state vector, we don't know yet the values of the x state
  ekf_.x_ = VectorXd(4);
  
  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
  
  //measurement matrix

  
  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
  
  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;


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
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      
      float rho = measurement_pack.raw_measurements_[0];
      float thi = measurement_pack.raw_measurements_[1];
      ekf_.x_ << rho*cos(thi), rho*sin(thi), 1, 1;
      previous_timestamp_ = measurement_pack.timestamp_;
      
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //set the state with the initial location and zero velocity
      
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 1, 1;
      previous_timestamp_ = measurement_pack.timestamp_;
      
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // TODO: YOUR CODE HERE
  //1. Modify the F matrix so that the time is integrated
  ekf_.F_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
  //2. Set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << (pow(dt,4)/4)*pow(noise_ax,1), 0, (pow(dt,3)/2)*pow(noise_ax,1), 0,
            0, (pow(dt,4)/4)*pow(noise_ay,1), 0, (pow(dt,3)/2)*pow(noise_ay,1),
            (pow(dt,3)/2)*pow(noise_ax,1), 0, pow(dt,2)*pow(noise_ax,1), 0,
            0, (pow(dt,3)/2)*pow(noise_ay,1), 0, pow(dt,2)*pow(noise_ay,1);
  
  //3. Call the Kalman Filter predict() function
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  
  // if both measures are zero, skip the update
  
  if (ekf_.x_(0)!=0 & ekf_.x_(0)!=0)
  {
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Radar updates
      ekf_.R_ = R_radar_;
      Hj_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.H_ = Hj_;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
      // Laser updates
      ekf_.R_ = R_laser_;
      ekf_.H_ = H_laser_;
      ekf_.Update(measurement_pack.raw_measurements_);
    }
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
