#include<bits/stdc++.h>
#include<kalman_filter.hpp>

KalmanFilter(uint8_t n, uint8_t m, uint8_t k) {
	n_ = n;
	m_ = n;
	k_ = k;
}
VectorXf KalmanFilter::set_initial(float pos, float velocity, float acc) {
	return VectorXf({ 0,0,0 });
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
MatrixXf KalmanFilter::predict_covariance() {
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
void KalmanFilter::call_equations() {
	x_ = set_initial();
	x_ = predict_state();
	P_ = predict_state_covariance();
	Kn_ = kalman_gain();
	x_ = estimate_state();
	P_ = estimate_state_covariance();
	update_state_transition();
}
int main() {
	return 0;
}