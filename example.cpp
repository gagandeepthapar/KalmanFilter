#include "eigen3/Eigen/Core"
// #include "matplot/matplot.h"
#include "matplotlibcpp.hpp"
#include "state_estimator.hpp"
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

namespace plt = matplotlibcpp;

int main() {

  // system timestep
  float DT = 0.01;
  int N = 1000;

  // mass spring dampener system
  int n = 2;
  double c_damp = 0.1;
  double k_spring = 1.0;
  double mass = 1.5;

  // State Space Matrices
  Eigen::MatrixXd A, C;

  A.resize(n, n);
  A(0, 0) = 0;
  A(0, 1) = 1;
  A(1, 0) = -k_spring / mass;
  A(1, 1) = -c_damp / mass;

  C.setIdentity(n, n);

  // KF Matrices
  Eigen::MatrixXd F, G, Q, H, R;

  F.setIdentity(n, n);
  F = A * DT + F;

  G.setZero(n, n);

  Q.resize(n, n);
  Q(0, 0) = 0.01;
  Q(1, 1) = 0.01;

  H.setIdentity(n, n);

  R.resize(n, n);
  R(0, 0) = 0.01;
  R(1, 1) = 0.01;

  KalmanFilter kf(F, G, Q, H, R);

  // create RNG for noise
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 0.1);

  // store true, measured, estimates
  Eigen::MatrixXd truth;
  truth.resize(n, n);
  std::vector<double> t, xT, xM, xE, dxT, dxM, dxE;
  Eigen::VectorXd x_0{{2.0, 0.0}};
  Eigen::VectorXd xTRUE{{2.0, 0.0}};
  Eigen::VectorXd x{{2.0, 0.0}};
  Eigen::VectorXd dx{{0.0, 0.0}};

  t.push_back(0.0);
  xT.push_back(x[0]);
  xM.push_back(x[0]);
  xE.push_back(x[0]);

  dxT.push_back(x[1]);
  dxM.push_back(x[1]);
  dxE.push_back(x[1]);

  Eigen::VectorXd u;
  u.resize(n, 1);
  u(0, 0) = 0;

  for (int i = 0; i < N; i++) {
    // system dynamics
    dx = A * xTRUE;
    xTRUE = dx * DT + xTRUE;

    xT.push_back(xTRUE[0]);
    dxT.push_back(xTRUE[1]);

    x = xTRUE;
    // add sensor noise
    x[0] += distribution(generator);
    x[1] += distribution(generator);

    xM.push_back(x[0]);
    dxM.push_back(x[1]);

    // pass through KF
    x = kf.update(x, u);
    xE.push_back(x[0]);
    dxE.push_back(x[1]);

    // repeat
    t.push_back(i * DT);
  }

  std::cout << kf.m_P << std::endl;

  plt::clf();
  plt::named_plot("True", t, xT, "--r");
  plt::named_plot("Measured", t, xM, "--b");
  plt::named_plot("Estimated", t, xE, "k");
  plt::legend();
  plt::show();

  return 0;
}
