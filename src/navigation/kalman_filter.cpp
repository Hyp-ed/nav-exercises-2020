#include <iostream>
#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter(uint8_t n, uint8_t m, uint8_t k) {
  n_ = n, m_ = m, k_ = k;
}

void KalmanFilter::set_initial(float dt) {
  x_ = VectorXf::Constant(n_, 0);
  A_ = createStateTransitionMatrix(dt);
  P_ = createStateCovarianceMatrix(kError);
  Q_ = createStateCovarianceNoiseMatrix(kStateCovarianceNoise);
  H_ = createDimensionChangeMatrix();
  R_ = createMeasurementNoiseMatrix(kMeasurementNoise);
  I_ = MatrixXf::Identity(n_, n_);
}

VectorXf KalmanFilter::get_state() {
  return x_;
}

MatrixXf KalmanFilter::get_covariance() {
  return P_;
}

void KalmanFilter::filter(VectorXf z) {
  z_ = z; // assigning measurement

  // 1. 
  predictState();

  // 2.
  estimateState();

  // 3.
  calculateKalmanGain();

  // 4.
  predictStateCovariance();

  // 5.
  estimateStateCovariance();
}

MatrixXf KalmanFilter::createStateTransitionMatrix(float dt) {
  MatrixXf A = MatrixXf::Constant(n_, n_, 0);
  A(0, 0) = 1;
  A(1, 1) = 1;
  A(2, 2) = 1;

  A(0, 1) = dt;
  A(1, 2) = dt;

  A(0, 2) = dt * dt / 2;

  return A;
}

MatrixXf KalmanFilter::createStateCovarianceMatrix(float val) {
  MatrixXf P = MatrixXf::Constant(n_, n_, val);
  return P;
}

MatrixXf KalmanFilter::createStateCovarianceNoiseMatrix(float val) {
  MatrixXf Q = MatrixXf::Constant(n_, n_, val);
  return Q;
}

MatrixXf KalmanFilter::createDimensionChangeMatrix() {
  MatrixXf H = MatrixXf::Constant(m_, n_, 0);
  for(int i = 0; i < m_; i++) {
    if(i < n_) {
      H(i, i) = 1;
    }
  }
  return H;
}

MatrixXf KalmanFilter::createMeasurementNoiseMatrix(float val) {
  MatrixXf R = MatrixXf::Constant(m_, m_, val);
  return R;
}

void KalmanFilter::predictState() {
  x_ = A_ * x_;
}

void KalmanFilter::estimateState() {
  x_ = x_ + K_ * (z_ - H_ * x_);
}

void KalmanFilter::calculateKalmanGain() {
  K_ = (P_ * H_.transpose()) * (H_ * P_ * H_.transpose() + R_).inverse();
}

void KalmanFilter::predictStateCovariance() {
  P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::estimateStateCovariance() {
  P_ = (I_ - K_ * H_) * P_;
}

void KalmanFilter::updateStateTransition(float dt) {
  A_ = createStateTransitionMatrix(dt);
}