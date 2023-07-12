#ifndef STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

// Abstract Base Class 
class StateEstimator{

public:
  virtual Eigen::VectorXd update(Eigen::VectorXd& measured_state, Eigen::VectorXd& input) = 0;


};


// Optimal Linear State Estimator 
// Derivation and Functions derived from 
  // "Spacecraft Dynamics and Control", DeRuiter
class KalmanFilter : public StateEstimator{

public:
  KalmanFilter(Eigen::MatrixXd& F_mat,
               Eigen::MatrixXd& G_mat,
               Eigen::MatrixXd& Q_mat,
               Eigen::MatrixXd& H_mat,
               Eigen::MatrixXd& R_mat);

  // KF is capable of updating the state w/o measurements
  Eigen::VectorXd update(Eigen::VectorXd& input);
  Eigen::VectorXd update(Eigen::VectorXd& measured_state, Eigen::VectorXd& input) override;
  void init_state(Eigen::VectorXd& x_0);
  void init_covar(Eigen::MatrixXd& P_0);

private:
  void init_sys();
  void prediction(Eigen::VectorXd& u);
  void correction(Eigen::VectorXd& y_meas);

public:
  Eigen::MatrixXd m_F; // state transition matrix
  Eigen::MatrixXd m_G; // effect of input on state transition
  Eigen::MatrixXd m_Q; // control noise
  Eigen::MatrixXd m_H; // measurement matrix 
  Eigen::MatrixXd m_R; // measurement noise
  Eigen::MatrixXd K; // optimal (Kalman) gain
  Eigen::MatrixXd m_P; // covariance matrix
  Eigen::MatrixXd m_PMinus; // prediction of covariance 
  Eigen::VectorXd m_xHat; // current best estimate of state
  Eigen::VectorXd m_xHatMinus; // best preduction of state
  Eigen::VectorXd m_yHat; // predicted state based on dynamics
  Eigen::MatrixXd W, W_pinv; // noise estimates

};

# endif // !STATE_ESTIMATOR_HPP
