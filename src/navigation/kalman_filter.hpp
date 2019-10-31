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
    void set_initial(float dt);
    // f7 : filter : Calls five main kalman filtering equations / functions
    void filter(VectorXf z);
    // f8 : get_state
    VectorXf get_state();
    // f9 : get_covariance
    MatrixXf get_covariance();

    // additional
    // f11 : create state transition matrix
    MatrixXf createStateTransitionMatrix(float dt); // A_ - done
    MatrixXf createStateCovarianceMatrix(float val); // P_
    MatrixXf createStateCovarianceNoiseMatrix(float val); // Q_
    MatrixXf createDimensionChangeMatrix(); // H_
    MatrixXf createMeasurementNoiseMatrix(float val); // R_ - done
    
  private:
    // f1 : predict state
    void predictState();
    // f2 : estimate state (update based on measurement)
    void estimateState();
    // f3 : kalman gain
    void calculateKalmanGain();
    // f4 : predict state covariance
    void predictStateCovariance();
    // f5 : estimate state covariance
    void estimateStateCovariance();
    // f10 : update state transition
    void updateStateTransition(float dt);
    
    static constexpr float kStateCovarianceNoise = 0.01; // Q_
    static constexpr float kMeasurementNoise = 0.01; // R_
    static constexpr float kError = 0.5; // New added - P_

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
    MatrixXf K_;  // Kalman gain matrix
};
#endif  // KALMAN_FILTER_HPP
