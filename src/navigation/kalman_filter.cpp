#include <bits/stdc++.h>
#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter(uint8_t n, uint8_t m, uint8_t k) {
	n_ = n;
	m_ = n;
	k_ = k;
}
VectorXf KalmanFilter::set_initial(VectorXf x) {
	return x;
}
MatrixXf KalmanFilter::set_initial_covariance(double cov) {
	uint8_t n = n_;
	MatrixXf P(n, n);
	for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++)
			P(i, j) = cov;
	return P;
}
VectorXf KalmanFilter::get_state() {
	return x_;
}
MatrixXf KalmanFilter::get_covariance() {
	return P_;
}
VectorXf KalmanFilter::predict_state() {
	VectorXf x;
	x = A_ * x_;// + B_ * z_;
	return x;
}
MatrixXf KalmanFilter::predict_state_covariance() {
	MatrixXf P;
	P = A_ * P_ * A_.transpose() + Q_;
	return P;
}
MatrixXf KalmanFilter::kalman_gain() {
	MatrixXf Kn;
	Kn = H_ * P_ * H_.transpose() * ((H_ * P_ * H_.transpose()) + R_).inverse();
	return Kn;
}
VectorXf KalmanFilter::estimate_state() {
	VectorXf x;
	x = x_ + (Kn_ * (z_ - (H_ * x_)));
	return x;
}
MatrixXf KalmanFilter::estimate_state_covariance() {
	MatrixXf P;
	P = (I_ - (Kn_ * H_)) * P_;
	return P;
}
MatrixXf KalmanFilter::update_state_transition(double dt) {
	uint8_t n = n_;
	MatrixXf A(n, n);
	for(int i = 0; i < n; i++)
		for (int j = 0; j < n; j++) {
			if (i == j) A(i,j) = 1;
			else
				if (j > i) {
					uint8_t ex = j - i;
					A(i, j) = pow(dt, ex) / ex;
				}
				else A(i, j) = 0;
		}
}
void KalmanFilter::call_equations(double dt, VectorXf x) {
	A_ = KalmanFilter::update_state_transition(dt);
	x_ = KalmanFilter::set_initial(x);
	P_ = KalmanFilter::set_initial_covariance(kStateCovarianceNoise);
	x_ = KalmanFilter::predict_state();
	P_ = KalmanFilter::predict_state_covariance();
	Kn_ = KalmanFilter::kalman_gain();
	x_ = KalmanFilter::estimate_state();
	P_ = KalmanFilter::estimate_state_covariance();
}
int main() {
	return 0;
}