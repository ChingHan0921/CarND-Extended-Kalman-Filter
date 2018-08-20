//Defines the predict function, the update function for lidar, 
//and the update function for radar.(Need to modify)
//Fill out the Predict(),Update(), and UpdateEKF() function.      

#include "kalman_filter.h"

#define PI 3.14159265

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  x_ = x_in; //x_:state vector
  P_ = P_in; //P_:state covariance matrix
  F_ = F_in; //F_:state transistion matrix
  H_ = H_in; //H_:measurement matrix
  R_ = R_in; //R_:measurement covariance matrix
  Q_ = Q_in; //Q_:process covariance matrix
}

void KalmanFilter::Predict() 
{
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) 
{
  /**
  TODO:
    * update the state by using Kalman Filter equations.(lidar)
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd k = PHt * Si;

  //New estimate
  x_ = x_ + (k * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size); 
  P_ = (I - k * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) 
{
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations.(radar)
  */
  //x_(0):Px, x_(1):Py, x_(2):Vx, x_(3):Vy

  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi = atan2(x_(1), x_(0));
  float rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;

  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;

  VectorXd y = z - z_pred;
  if(y[1] > PI)
    y[1] -= 2.f*PI;
  if( y[1] < -PI )
    y[1] += 2.f*PI;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd k = PHt * Si;

  //New estimate
  x_ = x_ + (k * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size); 
  P_ = (I - k * H_) * P_;
  
}
