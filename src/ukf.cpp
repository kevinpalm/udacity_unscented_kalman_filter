#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  
  // Save state dimension
  n_x_ = x_.size();
  
  // Save augmented state dimension
  n_aug_ = n_x_ + 2;
  
  // Define spreading parameter for sigma points
  lambda_ = 3 - n_aug_;
  
  // Create sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_x_ + 1);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    
  if (!is_initialized_) {

    // first measurement... we'll assume velocity, yaw_angle,
    // and yaw_rate are 0 for start
    cout << "UKF: " << endl;
    x_ = VectorXd(5);
    x_ << 1, 1, 0, 0, 0;
    
    //save timestamp
    time_us_ = meas_package.timestamp_;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      
      // Calculate x position
      x_(0) = cos(fmod(meas_package.raw_measurements_(1), 2*3.1415926535897932))*meas_package.raw_measurements_(0);
      
      // Calculate y position
      x_(1) = sin(fmod(meas_package.raw_measurements_(1), 2*3.1415926535897932))*meas_package.raw_measurements_(0);
      
    }
    
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
		
	  // Fill x position
	  x_(0) = meas_package.raw_measurements_(0);
      
      // Fill y position
	  x_(1) = meas_package.raw_measurements_(1);
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  
  // Calculate the delta t and store old timestamp
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;
  
  // Predict
  Prediction(delta_t);
  
  // Update
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    
    // Radar update
    UpdateRadar(meas_package);
    
  } else {
      
      // Lidar update
      UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  
  
  /*********************************************************************
   * Augment sigma points
   ********************************************************************/

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
 
  //fill augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //fill augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //fill augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  /*********************************************************************
   * Predict sigma points
   ********************************************************************/
   
   
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
