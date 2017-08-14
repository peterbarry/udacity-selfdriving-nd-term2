#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


#define SMALL_NUMBER 0.001

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  ///* initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
    // Start P_ with identity  matric.

  P_ <<     1,    0,    0,    0,    0,
            0,    1,    0,    0,    0,
            0,    0,    1,    0,    0,
            0,    0,    0,    1,    0,
            0,    0,    0,    0,    1;


  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //create sigma point matrix TODO: Needed ? reused from class.
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  ///* time when the state is true, in us
  time_us_ = 0;


  ///* Sigma point spreading parameter
  double lambda_= 3 - n_aug_;

  ///* Weights of sigma points - reused from class.
  weights_ = VectorXd(2*n_aug_+1);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  // Process noise standard deviation longitudinal acceleration in m/s^2

  // too high for a bike std_a_ = 30;
  std_a_ = 0.3; //TODO: tune this value.

  // Process noise standard deviation yaw acceleration in rad/s^2
  // too high for a bike std_yawdd_ = 30;
  std_yawdd_ = 0.6; //TODO: Tune this value.

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


  NIS_radar_ = 0;
  NIS_lidar_ = 0;

  // Measurement noise covariance matrices initialization
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0,std_radrd_*std_radrd_;

  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_*std_laspx_,0,
              0,std_laspy_*std_laspy_;


}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  cout << "UKF: Process Measurement " << endl;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    cout << "Radar Measurement" << endl;
    if ( use_radar_ != true) {// do not process
      return;
    }
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    cout << "LASER Measurement" << endl;
    if ( use_laser_ != true) { // do not process.
      return;
    }
  }

  if (!is_initialized_) {
    cout << "EKF: Initialize " << endl;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
        float rho,phi,rhodot;
        float x,y,velx,vely,vel;

            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            rho = measurement_pack.raw_measurements_[0]; // range
            phi = measurement_pack.raw_measurements_[1]; // bearing
            rhodot = measurement_pack.raw_measurements_[2]; // velocity of rho

            // Coordinates convertion from polar to cartesian
            x = rho * cos(phi);
            y = rho * sin(phi);
            velx = rhodot * cos(phi);
            vely = rhodot * sin(phi);
            vel = sqrt(velx * velx + vely * vely);
            x_ << x, y, vel , 0, 0;
            previous_timestamp_ = measurement_pack.timestamp_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //set the state with the initial location and zero velocity
      // make sure no div 0.
      if (measurement_pack.raw_measurements_[0] != 0 && measurement_pack.raw_measurements_[1] != 0)
        x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0,0;
      else
        x_ << SMALL_NUMBER,SMALL_NUMBER,0,0,0;

      previous_timestamp_ = measurement_pack.timestamp_;
    }
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;

  }

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;     //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  Prediction(dt);


  // Update, different update for radar and lidar.

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
	   cout << "Radar Update:" << measurement_pack.raw_measurements_[0] << " " << measurement_pack.raw_measurements_[1] << endl;
     UpdateRadar(measurement_pack);
  }
  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
	  cout << "Lidar Update:" << measurement_pack.raw_measurements_[0] << " " << measurement_pack.raw_measurements_[1] << endl;
    UpdateLidar(measurement_pack);
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
  // Gen Sigma Points
  GenerateAugmentedSigmaPoints();
  cout << "  Sigma Points = " << Xsig_aug_ << endl;


  PredictSigmaPoints(delta_t);
  PredictMeanAndCovariance();

}


void UKF::PredictSigmaPoints(double delta_t) {

  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  //print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred_ << std::endl;

}

void UKF::PredictMeanAndCovariance() {

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);
  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x = x+ weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }

  x_ = x;
  P_ = P;

}

// reused from class material
void UKF::GenerateAugmentedSigmaPoints() {

  //create augmented mean vector

  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  //create augmented state covariance

  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug_.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
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
  int n_z = 2;

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

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;


}
