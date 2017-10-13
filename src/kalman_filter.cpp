#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */

	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

	VectorXd z_prep= KalmanFilter::ConverttoRadar(x_);
	VectorXd y= z-z_prep;
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

	while (y(1) > ( M_PI)) {
	          y(1) = y(1)-(2*M_PI);
	      }

	  while (y(1) < -(M_PI)) {
	          y(1) = y(1)+ (2*M_PI);
	      }



	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;




}

VectorXd KalmanFilter::ConverttoRadar(const VectorXd &x_state){

	VectorXd x_radar(3);
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float c1= px*px+ py*py;


	x_radar(0)= std::sqrt(c1);

	x_radar(1)= std::atan2(py,px);
	x_radar(2)= (px*vx+py*vy)/std::sqrt(c1);
	 while (x_radar(1) > ( M_PI)) {
	        x_radar(1) = x_radar(1)-(2*M_PI);
	      }

	  while (x_radar(1) < -(M_PI)) {
	          x_radar(1) = x_radar(1)+ (2*M_PI);
	      }

	   //check division by zero
	  if (x_radar(0)<.0001){
	      x_radar(2) = (px*vx+py*vy)/0.0001;
	      }
	  else{
	      x_radar(2) = (px*vx+py*vy)/x_radar(0);
	  }


	return x_radar;
















}
