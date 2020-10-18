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

 

  

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.8;

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
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  //Initializing/Setting all the values with a this pointer as multiple UKF objects; one for each car has to be loaded
  //Values not been initialized so setting this member variable as false
  this->is_initialized_ = false;

  // Total number of dimensions for a single state for the CTRV model
  this->n_x_=5;
  // Total number of dimensions after applying augmentation(considering noise longitudinal and yaw acceleration)
  this->n_aug_=7;
  // Total number of dimensions for Radar in measurement space
  this->n_z = 3;
  // Total number of dimensions for Lidar in measurement space
  this->n_l=2;

  //lambda which is a factor that controls the spreading of the Sigma points
  this->lambda_=3-(this->n_aug_);

  // initial state vector of the size n_x(number of dimensions)
  this->x_ = VectorXd(this->n_x_);
  
  
  // initial covariance matrix 
  this->P_ = MatrixXd(this->n_x_, this->n_x_);
  
  //setting the diagonal values by how much difference you expect between the true state and the initialized x state vector

  // this->P_ << std_laspx_*std_laspx_, 0, 0, 0, 0,
  // 				0, std_laspy_*std_laspy_, 0, 0, 0,
  // 				0, 0, std_radrd_*std_radrd_, 0, 0,
  // 				0, 0, 0, std_radphi_*std_radphi_, 0,
  // 				0, 0, 0, 0, 1;
  
   P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, std_laspx_*std_laspy_, 0,
        0, 0, 0, 0, std_laspx_*std_laspy_;

  // Initializing weights to recover mean and covariance from the predicted Sigma points
  // Somehow weights depends on lamda parameter(Controls Sigma points spread)

  // length of the weights vector is equal to the number of sigma points 
  this->weights_ = VectorXd((2*this->n_aug_) + 1);

  //Setting weights for first sigma point(which is generally mean) and rest of the remain pairs of Sigma points(one for each dimension)
  this->weights_.fill(0.5/(this->n_aug_ + this->lambda_));

  //Resetting the first weight(corresponding to the mean) as the formula for that is mentioned below
  this->weights_(0) = this->lambda_/(this->lambda_ + this->n_aug_);

  // Sigma points prediction matrix Initialized has n_x rows(number of dimensions of the State vector) and n_aug*2 +1 columns
  this->Xsig_pred_ = MatrixXd(this->n_x_, (this->n_aug_*2)+1);
  
  //Initialize Process Noise covariance
  this->Q = MatrixXd(2, 2);
  this->Q << std_a_*std_a_,0,
            0,std_yawdd_*std_yawdd_;
  // Initialize Measurement Noise covariance matrix for measurement space for both Radar and Lidar
  this->R_Radar_ = MatrixXd(this->n_z, this->n_z);
  this->R_Radar_ << std_radr_*std_radr_, 0, 0,
                    0, std_radphi_*std_radphi_, 0,
                    0, 0, std_radrd_*std_radrd_;

  this->R_Lidar_ = MatrixXd(this->n_l,this->n_l);
  this->R_Lidar_ << std_laspx_*std_laspx_, 0,
                    0, std_laspy_*std_laspy_;

   // the current NIS(Normalized Innovation Square) for radar/initialization
   this->NIS_Radar_ = 0.0;

   // the current NIS(Normalized Innovation Square) for laser/initialization
   this->NIS_Laser_ = 0.0;
}


UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  /**
   * step 1. Check whether it is the first Measurement or not
   * yes -> Then initialize State x and Covariance P
   * No  -> Or else go straight to Prediction
   */

  if(!this->is_initialized_){
    this->x_.fill(0.0); //Initialise state vector with all zeros

    //Check the sensor type RADAR or Lidar
    if(meas_package.sensor_type_== MeasurementPackage::RADAR)
    {
      this->x_[0] = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
      this->x_[1] = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
      double vx = meas_package.raw_measurements_[2] * cos(meas_package.raw_measurements_[1]);
      double vy = meas_package.raw_measurements_[2] * sin(meas_package.raw_measurements_[1]);
      this->x_[2] =  sqrt(vx * vx + vy * vy);
    }
    else if (meas_package.sensor_type_== MeasurementPackage::LASER)
    {
       this->x_[0] = meas_package.raw_measurements_[0];
       this->x_[1] = meas_package.raw_measurements_[1];
    }
    else
    {
      std::cout << "Invalid sensor type" << std::endl;
      return;
    }
    //Initialization done
    this->is_initialized_ = true; 
    //Starting time initialization
    this->time_us_ = meas_package.timestamp_;
    return;
  }

  /**
   * step 2. Prediction
   * a. Compute the elapsed time dt or delta t
   * b. Compute the augmented Mean(x_aug) and Covariance(P_aug) after considering process noise vector component 
   *    which having additional two elements of 2D filtering process, those are longitudinal and yaw process noise vector components  
   * c. Compute the x_k|k, P_k|k
   */

  else
  {
    //if the initialization is done, continue the prediction
    //update the delta t and the current time
    double dt = (meas_package.timestamp_ - this->time_us_) / 1000000.0;
    this->time_us_ = meas_package.timestamp_;
    //start the prediction
    Prediction(dt);

    /**
   * step 3. Check whether if the received data is of Radar or Lidar
   * If Radar-> set the measurement vector z = new Radar reading from MeasurementPackage
   * If Lidar-> set the measurement vector z = new Lidar reading from  MeasurementPackage
   * a. calculate the pridicted measurement mean and covariance z_k|k, S from x_k|k, P_k|k
   * b. calculate the Kalman gain K for the ukf instance, and also the difference between the real and predicted meansurements
   * z_diff = z - z_k|k
   * c. Further calculate the new estimated state vector and covariance matrix/uncertainty x_k+1|k, P_k+1|k through Uncented Kalman filter
   */
    
    //At last you need to update the state based on the sensor data type

    if(this->use_radar_ == true && meas_package.sensor_type_ == MeasurementPackage::RADAR) UpdateRadar(meas_package);
    if(this->use_laser_ == true && meas_package.sensor_type_ == MeasurementPackage::LASER) UpdateLidar(meas_package);

   
  
  }

   
  
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

   // creating augmented mean state x_aug and intialising it to zero and filling the head of the vector with state vector received from measurement package
   // of the sensor
  Eigen::VectorXd x_aug = VectorXd(this->n_aug_);
  x_aug.fill(0.0);
  x_aug.head(this->n_x_) = this->x_;
  

  // creating augmented covariance matrix P_aug and intialising it to zero first and then filling the top left corner of the
  // matrix with covariance matrix set 
  Eigen::MatrixXd P_aug = MatrixXd(this->n_aug_ , this->n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(this->n_x_,this->n_x_) = this->P_;

  // add Q matrice or the Process noise matrix  to the bottom right corner of augmented covariance matrix P_aug 
  P_aug(this->n_x_, this->n_x_) = pow(this->std_a_, 2);
  P_aug(this->n_x_ +1, this->n_x_ +1) = pow(this->std_yawdd_, 2);

  // creates the  square root matrix using cholesky_module that returns the LLT decomposition
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points using lamda parameter that controls the spread of the approximated gaussian 
  Eigen::MatrixXd Xsig_aug = MatrixXd(this->n_aug_ , 2*this->n_aug_ +1);
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< this->n_aug_; ++i) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(this->lambda_ + this->n_aug_) * L.col(i);
    Xsig_aug.col(i+1+ this->n_aug_) = x_aug - sqrt(this->lambda_ + this->n_aug_) * L.col(i);
  }

  // predicting sigma points 
  for (int i = 0; i< 2*this->n_aug_ +1; ++i) {
    // extracting values for a better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;
    
    
    // avoiding division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }
    
    // predicted velocity magnitude v_p and angle yaw_p and angular change rate yawd_p
    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // adding process noise to the predicting elements of the state vector
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into the related column
    this->Xsig_pred_(0,i) = px_p;
    this->Xsig_pred_(1,i) = py_p;
    this->Xsig_pred_(2,i) = v_p;
    this->Xsig_pred_(3,i) = yaw_p;
    this->Xsig_pred_(4,i) = yawd_p;
  }

  // predicted state mean vectors for all sigma points with weight of each sigma point in consideration
  this->x_ = this->Xsig_pred_ * this->weights_;

  // predicted state covariance matrix
  this->P_.fill(0.0);
  for (int i = 0; i < 2 * this->n_aug_ + 1; ++i) {  // loop over all the sigma points and update the predicted covariance matrix
    // state difference
    VectorXd x_diff = this->Xsig_pred_.col(i) - this->x_;
    // normalization of the angle
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    this->P_ = this->P_ + this->weights_(i) * x_diff * x_diff.transpose() ;
  }

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  
  // vector for storing the incoming lidar measurements
  VectorXd z = VectorXd(this->n_l);
  //add the incoming lidar reading to the z vector
  double px = meas_package.raw_measurements_(0);
  double py = meas_package.raw_measurements_(1);
  z << px, py;
  
  // transform all of the sigma points to the  measurement space

  // create a vector for mean predicted measurement
  VectorXd z_pred = VectorXd(this->n_l); 
  // mean predicted measurement 
  z_pred = this->x_.head(n_l);

  // S is the innovation covariance matrix 
  // This Matrix is also predicted measurement covariance
  MatrixXd S = MatrixXd(this->n_l,this->n_l);

  // add the Lidar measurement noise covariance matrix to S
  S = this->P_.topLeftCorner(this->n_l,this->n_l) + this->R_Lidar_;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(this->n_x_ , this->n_l);

  // calculate cross correlation matrix looping over all the sigma points
  Tc.fill(0.0);
  for (int i = 0; i < 2 * this->n_aug_ + 1; ++i) {  // 2n+1 simga points

    // state difference
    VectorXd x_diff = this->Xsig_pred_.col(i) - this->x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    // residual vector holding the difference between the prediction/estimation and the ground truth values
    VectorXd z_diff = x_diff.head(this->n_l);

    Tc = Tc + (this->weights_(i) * x_diff * z_diff.transpose());
  }

  //Calculate Kalman/filter gain K;
  MatrixXd K = Tc * S.inverse();

  // residual vector has to be calculate again as the z_diff inside the for is local to that loop only
  VectorXd z_diff = z - z_pred;
   
  // Final step is to upate based on the ground truth and prediction difference along with kalman gain 
  // update state mean and covariance matrix
  this->x_ = this->x_ + K * z_diff;
  this->P_ = this->P_ - K*S*K.transpose();
  
  //NIS (Normalized Innovation Square)  calculation for Lidar
  this->NIS_Laser_ = z_diff.transpose() * S.inverse() * z_diff;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  //Initialise the state vector with respect to  the radar data size
  // Creating a vector for incoming radar measurement
  VectorXd z = VectorXd(this->n_z);
  //add the incoming radar reading to z vector
  double rho = meas_package.raw_measurements_(0);
  double phi = meas_package.raw_measurements_(1);
  double rho_dot = meas_package.raw_measurements_(2);
  z << rho, phi, rho_dot;
  
  // transforming sigma points to the measurement space
  // create a matrix with sigma points in measurement space
  MatrixXd Zsig = MatrixXd(this->n_z, 2 * this->n_aug_ + 1);
  // create a vector for mean predicted measurement
  VectorXd z_pred = VectorXd(this->n_z);
  
  for (int i = 0; i < 2 * this->n_aug_ + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = this->Xsig_pred_(0,i);
    double p_y = this->Xsig_pred_(1,i);
    double v  = this->Xsig_pred_(2,i);
    double yaw = this->Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model for Radar
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r(radial distance)
    Zsig(1,i) = atan2(p_y,p_x);                                // phi(angle)
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot(anglular rate)
  }

  // mean predicted measurement
  z_pred = Zsig * this->weights_;

  // innovation covariance matrix S
  // create a matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(this->n_z,this->n_z);
  S.fill(0.0);
  for (int i = 0; i < (2 * this->n_aug_) + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + this->weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix

  S = S + this->R_Radar_;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(this->n_x_ , n_z);

  // calculate cross correlation matrix which is used to calculate the filter/Kalman Gain
  Tc.fill(0.0);
  for (int i = 0; i < 2 * this->n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual vector the difference between the predicted state of each sigma point wrt to the ground truth
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = this->Xsig_pred_.col(i) - this->x_;
    // Normalization of angle
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + this->weights_(i) * x_diff * z_diff.transpose();
  }

  // UKF filter's/Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual vector has to be calculate again as the z_diff inside the for is local to that
  VectorXd z_diff = z - z_pred;

  // Normalization of angle 
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // Consider filter gain update state mean and covariance matrix
  this->x_ = this->x_ + K * z_diff;
  this->P_ = this->P_ - K*S*K.transpose();

  //NIS (Normalized Innovation Square) calculation for Radar
  this->NIS_Radar_ = z_diff.transpose() * S.inverse() * z_diff;
}