#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP
#include <iostream>
#include <Eigen/Dense>

namespace hyped {

using Eigen::VectorXf;
using Eigen::MatrixXf;

namespace navigation {

class KalmanFilter {
  public:
    /*
     * Constructor call for a kalman filter
     * n : dimension of the state
     * m : dimension of the measurement
     * k : dimension of control   NOTE: will not be used in this implementation
     */
    KalmanFilter(uint8_t = 3, uint8_t m = 1, uint8_t k = 0);

    // f6 : set_initial
    // f7 : filter : Calls five main kalman filtering equations / functions
    // f8 : get_state
    // f9 : get_covariance

  private:
    // f1 : predict state
    // f2 : estimate state (update based on measurement)
    // f3 : kalman gain
    // f4 : predict state covariance
    // f5 : estimate state covariance
    // f10 : update state transition
    static constexpr float kStateCovarianceNoise = 0.01;
    static constexpr float kMeasurementNoise = 0.01;

    uint8_t n_;
    uint8_t m_;
    uint8_t k_;

    VectorXf x_;  // current state (n x 1)
    VectorXf z_;  // measurement (m x 1)
    MatrixXf A_;  // state transition matrix (n x n)
    MatrixXf P_;  // state covariance matrix (n x n)
    MatrixXf Q_;  // state covariance noise matrix (n x n)
    MatrixXf H_;  // dimension change matrix (m x n)
    MatrixXf R_;  // measurement noise matrix (m x m)
    MatrixXf I_;  // identity matrix (n x n)

};
}}
#endif  // KALMAN_FILTER_HPP
