#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP
#include <iostream>
#include <Eigen/Dense>

using Eigen::VectorXf;
using Eigen::MatrixXf;

class KalmanFilter {
  public:
    /*
     * Constructor call for a kalman filter
     * n : dimension of the state
     * m : dimension of the measurement
     * k : dimension of control   NOTE: will not be used in this implementation
     */
    KalmanFilter(uint8_t n, uint8_t m, uint8_t k = 0);
	
    // f6 : set_initial
	VectorXf set_initial(float pos, float velocity, float acc);
    // f7 : filter : Calls five main kalman filtering equations / functions
	void call_equations();
    // f8 : get_state
	VectorXf get_state();
    // f9 : get_covariance
	MatrixXf get_covariance();

  private:
    // f1 : predict state
	  VectorXf predict_state();
	// f4 : predict state covariance
	  MatrixXf predict_state_covariance();

	// f3 : kalman gain
	  VectorXf kalman_gain();

    // f2 : estimate state (update based on measurement)
	  VectorXf estimate_state();
	// f5 : estimate state covariance
	  MatrixXf estimate_state_covariance();

    // f10 : update state transition
	  MatrixXf update_state_transition();

    static constexpr float kStateCovarianceNoise = 0.01;
    static constexpr float kMeasurementNoise = 0.01;

    uint8_t n_;
    uint8_t m_;
    uint8_t k_;

    VectorXf x_;  // current state (n x 1)
    VectorXf z_;  // measurement (m x 1)
    MatrixXf A_;  // state transition matrix (n x n)
	MatrixXf B_;  // matrix that transforms my input into usable data
    MatrixXf P_;  // state covariance matrix (n x n)
    MatrixXf Q_;  // state covariance noise matrix (n x n)
    MatrixXf H_;  // dimension change matrix (m x n)
    MatrixXf R_;  // measurement noise matrix (m x m)
    MatrixXf I_;  // identity matrix (n x n)
	MatrixXf Kn_; // Kalman gain

};
#endif  // KALMAN_FILTER_HPP
