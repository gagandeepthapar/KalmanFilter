#include "state_estimator.hpp"
#include <iostream>


// Kalman Filter
KalmanFilter::KalmanFilter(Eigen::MatrixXd& F_mat,
                           Eigen::MatrixXd& G_mat,
                           Eigen::MatrixXd& Q_mat,
                           Eigen::MatrixXd& H_mat,
                           Eigen::MatrixXd& R_mat):
                           m_F(F_mat),
                           m_G(G_mat),
                           m_Q(Q_mat),
                           m_H(H_mat),
                           m_R(R_mat){

  init_sys();

}

void KalmanFilter::init_sys(){
  // add large covariance to P and P-
  Eigen::MatrixXd covar;
  covar.setIdentity(this->m_F.rows(), this->m_F.cols());
  covar = covar * 100.0;

  // resize and fill with zeros
  this->K.setZero(this->m_F.rows(), this->m_F.cols()); 
  // this->m_P = Eigen::MatrixXd
  this->m_P.setZero(this->m_F.rows(), this->m_F.cols()) + covar;
  this->m_PMinus.setZero(this->m_F.rows(), this->m_F.cols()) + covar;
  this->m_xHat.setZero(this->m_F.rows(), 1);
  this->m_xHatMinus.setZero(this->m_F.rows(), 1);
  this->m_yHat.setZero(this->m_F.rows(), 1);
  this->W.setZero(this->m_F.rows(), this->m_F.cols());
  this->W_pinv.setZero(this->m_F.rows(), this->m_F.cols());

}

void KalmanFilter::init_state(Eigen::VectorXd& x_0){
  this->m_xHat = x_0;
}

void KalmanFilter::init_covar(Eigen::MatrixXd& P_0){
  this->m_P = P_0;
}

void KalmanFilter::prediction(Eigen::VectorXd& u){
  // Prediction step of Kalman Filter 
  this->m_xHatMinus = this->m_F * this->m_xHat + this->m_G * u;
  this->m_PMinus = this->m_F * this->m_P * this->m_F.transpose() + this->m_Q;

}

void KalmanFilter::correction(Eigen::VectorXd& y_meas){
  // Correction step of Kalman Filter
  this->W = this->m_H * this->m_PMinus * this->m_H.transpose() + this->m_R;

  // calc psuedo-inverse to escape non-invertability case
  this->W_pinv = (W * W.transpose()).inverse() * W.transpose();
  
  // calc Kalman gain
  this->K = this->m_PMinus * this->m_H.transpose() * W_pinv;

  // calc expected state
  this->m_yHat = this->m_H * this->m_xHatMinus + this->m_R.diagonal(); 
  
  // correct state estimate and covariance 
  this->m_xHat = this->m_xHatMinus + this->K * (y_meas - this->m_yHat);

  this->m_P = (this->m_PMinus - this->K * this->m_H * this->m_PMinus) + 
              (-1*this->m_PMinus * this->m_H.transpose() * this->K.transpose()) +
              (this->K * W * this->K.transpose());

}

Eigen::VectorXd KalmanFilter::update(Eigen::VectorXd& input){
  // return prediction of state  
  this->prediction(input);
  
  // set covar and state to best estimate (prediction)
  this->m_xHat = this->m_xHatMinus;
  this->m_P = this->m_PMinus;

  // return best estimate
  return this->m_xHatMinus;
}


Eigen::VectorXd KalmanFilter::update(Eigen::VectorXd& measured_state, Eigen::VectorXd& input){
  // Return corrected estimate 
  this->prediction(input);
  this->correction(measured_state);
  return this->m_xHat;
}


