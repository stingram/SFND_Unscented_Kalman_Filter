#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  n_x_ = 5;


  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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

   H_ = MatrixXd(2, 5);
      H_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0;


  R_ = MatrixXd(2, 2);
      R_ << std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_ ;

  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  x_ = VectorXd(n_x_);
  x_ << 1,1,1,1,1;
  P_ = Eigen::MatrixXd(5,5);
  
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 
        0, 0 , 1, 0, 0,
        0, 0,  0,  0.0225, 0,
        0, 0,  0,  0, .0225;

 
  is_initialized_ = false;
   
  // Set initial values for x_
  n_x_ = 5;
  n_aug_ = 7;
  
  weights_ = VectorXd(2*n_aug_+1);
  weights_.fill(0.0);
    
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);
  
  H_ = MatrixXd(2, 5);
  H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;
  
  R_ = MatrixXd(2, 2);
  R_ << std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_ ;

  lambda_ = 0;
  time_us_ = 0;

  NIS_radar = 0;
  NIS_laser = 0; 

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  if(is_initialized_)
  {
      // Compute delta_t
      float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
      time_us_ = meas_package.timestamp_;

      // First call predict
      Prediction(delta_t);

      // Either Update Radar or/and Lidar
      if(meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR && use_radar_)
      {
        UpdateRadar(meas_package);
      }
      if(meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER && use_laser_)
      {
        UpdateLidar(meas_package);
      }
  }
  else
  {
      // Either Initialize Radar or/and Lidar
      if(meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR && use_radar_)
      {
          double rho = meas_package.raw_measurements_(0);
          double phi = meas_package.raw_measurements_(1);
          double rhodot = meas_package.raw_measurements_(2);
          double x = rho*cos(phi);
          double y = rho*sin(phi);
          double vx = rhodot*cos(phi);
          double vy = rhodot*sin(phi);
          double v = sqrt(vx*vx+vy*vy);

          x_ << x,
                y,
                0,
                0,
                0;
      }
      else if(meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER && use_laser_)
      {
          x_ << meas_package.raw_measurements_(0),
                meas_package.raw_measurements_(1),
                0,
                0,
                0;
      }
      is_initialized_ = true;
      time_us_ = meas_package.timestamp_;
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
      
    // GENERATE SIGMA POINTS WITH AUGMENTATION
    lambda_ = 3 - n_aug_;

    // create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    Xsig_aug.fill(0.0);
  
    // create augmented mean vector
    VectorXd x_aug = VectorXd(7);
    x_aug.fill(0.0);

    // create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);
    P_aug.fill(0.0);
  
    MatrixXd Q = MatrixXd(2, 2);
    Q << pow(std_a_,2), 0, 0, pow(std_yawdd_,2);
   
    // create augmented mean state
    x_aug.head(n_x_) = x_;
  
    // create augmented covariance matrix
    P_aug.topLeftCorner(n_x_,n_x_) = P_;
    P_aug.bottomRightCorner(n_aug_-n_x_,n_aug_-n_x_) = Q;

    // create square root matrix
    MatrixXd A_aug = P_aug.llt().matrixL();

    // create augmented sigma points
    Xsig_aug.col(0) = x_aug;
  
    for(int i=0;i<P_aug.cols();i++)
    {
      Xsig_aug.col(i+1) = x_aug + std::pow((lambda_+n_aug_),0.5)*A_aug.col(i);
    }
    for(int i=0;i<P_aug.cols();i++)
    {
      Xsig_aug.col(n_aug_+i+1) = x_aug - std::pow((lambda_+n_aug_),0.5)*A_aug.col(i);
    }

  // SIGMA POINT PREDICTION
  // predict sigma points
  // avoid division by zero
  // write predicted sigma points into right column
  float dt = delta_t;
  for(int i=0;i<Xsig_aug.cols();i++)
  {
	  float vel        = Xsig_aug(2, i);
	  float phi        = Xsig_aug(3, i);
	  float phi_dot    = Xsig_aug(4, i);
	  float noise_acc  = Xsig_aug(5, i);
	  float noise_yaw  = Xsig_aug(6, i);
	  MatrixXd process = MatrixXd(5, 1);
	  MatrixXd noise   = MatrixXd(5, 1); 

    if(abs(phi_dot) > 0.001)
	  {
            
		process << vel/phi_dot*(sin(phi+phi_dot*dt)-sin(phi)),
		           vel/phi_dot*(-cos(phi+phi_dot*dt)+cos(phi)),
		           0,
		           phi_dot*dt,
		           0;
		noise << 0.5*pow(dt,2)*cos(phi)*noise_acc,
		         0.5*pow(dt,2)*sin(phi)*noise_acc,
		         dt*noise_acc,
		         0.5*pow(dt,2)*noise_yaw,
		         dt*noise_yaw;
		Xsig_pred_.col(i) = Xsig_aug.block(0,i,5,1) + process + noise;
	  }
	  else
	  {
		process << vel*cos(phi)*dt,
		           vel*sin(phi)*dt,
		           0,
		           phi_dot*dt,
		           0;
		noise << 0.5*pow(dt,2)*cos(phi)*noise_acc,
		         0.5*pow(dt,2)*sin(phi)*noise_acc,
		         dt*noise_acc,
		         0.5*pow(dt,2)*noise_yaw,
		         dt*noise_yaw;
		Xsig_pred_.col(i) = Xsig_aug.block(0,i,5,1) + process + noise;		  
	  }	   
  }
  
  // PREDICT STATE MEAN AND COVARIANCE
  VectorXd x_pred = VectorXd(n_x_);
  MatrixXd P_pred = MatrixXd(n_x_, n_x_);

  x_pred.fill(0.0);
  P_pred.fill(0.0);


  // set weights
  for(int i=0;i<weights_.rows();i++)
  {
      if(i == 0)
      {
          weights_(i) = lambda_/(lambda_+n_aug_);
      }
      else
      {
          weights_(i) = 1./(2*(lambda_+n_aug_));
      }
  }
  // predict state mean
  for(int i=0;i<Xsig_pred_.cols();i++)
  { 
      x_pred += weights_(i)*Xsig_pred_.col(i);
  }

  // predict state covariance matrix
  for(int i=0;i<Xsig_pred_.cols();i++)
  {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_pred;

    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_pred += weights_(i) * x_diff * x_diff.transpose();
  }
  x_ = x_pred;
  P_ = P_pred;
  
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  // PREDICT LIDAR MEASUREMENT

  int n_z = 2;
  
    // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  
  // transform sigma points into measurement space
  for(int i=0;i<Xsig_pred_.cols();i++)
  {
      float px = Xsig_pred_(0,i);
      float py = Xsig_pred_(1,i);
      Zsig.col(i) << px, py;
  }
  
  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for(int i=0;i<Zsig.cols();i++)
  {
      z_pred += weights_(i)*Zsig.col(i);
  }
  // calculate innovation covariance matrix S
  S.fill(0.0);
  for(int i=0;i<Zsig.cols();i++)
  {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
   S(0,0) += pow(std_laspx_,2);
   S(1,1) += pow(std_laspy_,2);

  // UPDATE STATE
  
  // calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for(int i=0;i<Zsig.cols();i++)
  {
      // residual
      VectorXd z_diff = Zsig.col(i) - z_pred;
    
      // state difference
      Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
      
      // angle normalization
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
      Tc += weights_(i)*(x_diff)*(z_diff).transpose();
  }

  // calculate Kalman gain K;
  MatrixXd K = Tc*S.inverse();
  
  // get z from measurement
  VectorXd z = meas_package.raw_measurements_;
  
  // residual
  Eigen::VectorXd z_diff = z - z_pred;

  // update state mean and covariance matrix
  x_ += K*(z_diff);
  P_ -= K*S*K.transpose(); 
  
   
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  
  // PREDICT RADAR MEASUREMENT
  int n_z = 3;
  
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  // transform sigma points into measurement space
  for(int i=0;i<Xsig_pred_.cols();i++)
  {
      double px  = Xsig_pred_(0,i);
      double py  = Xsig_pred_(1,i);
      double v   = Xsig_pred_(2,i);
      double yaw = Xsig_pred_(3,i);
      double v1  = v*cos(yaw);
      double v2  = v*sin(yaw);
    
      double ro     = sqrt(px*px + py*py);
      double eta    = atan2(py, px);
      double ro_dot = (px*v1+py*v2)/std::max(0.0001,ro);
      Zsig.col(i) << ro, eta, ro_dot;
  }
  
  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for(int i=0;i<Zsig.cols();i++)
  {
      z_pred += weights_(i)*Zsig.col(i);
  }
  // calculate innovation covariance matrix S
  S.fill(0.0);
  for(int i=0;i<Zsig.cols();i++)
  {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
   S(0,0) += pow(std_radr_,2);
   S(1,1) += pow(std_radphi_,2);
   S(2,2) += pow(std_radrd_,2);
  
  // UPDATE STATE
  
  // calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for(int i=0;i<Zsig.cols();i++)
  {
      // residual
      VectorXd z_diff = Zsig.col(i) - z_pred;

      // angle normalization
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
      // state difference
      Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
      
    // angle normalization
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
      Tc += weights_(i)*(x_diff)*(z_diff).transpose();
  }

  // calculate Kalman gain K;
  MatrixXd K = Tc*S.inverse();
  
  // get z from measurement
  VectorXd z = meas_package.raw_measurements_;
  
  // residual
  Eigen::VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x_ += K*(z_diff);
  P_ -= K*S*K.transpose();
 

}