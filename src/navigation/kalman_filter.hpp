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
      // f7 : filter : Calls five main kalman filtering equations / functions
      // f8 : get_state
      // f9 : get_covariance

      void set_initial(VectorXf init);

      VectorXf get_state();

      MatrixXf get_covariance();

      void filter();

    private:
      // f1 : predict state

      VectorXf predict_state();
      // f2 : estimate state (update based on measurement)
      VectorXf estimate_state(MatrixXf K);
      // f3 : kalman gain
      MatrixXf kalman_gain();
      // f4 : predict state covariance
      MatrixXf predict_covariance();
      // f5 : estimate state covariance
      MatrixXf estimate_covariance(MatrixXf K);
      // f10 : update state transition;
      void update_state_transition(double Dt);
      // f11 : update state transition;
      void update_sensor_noise(MatrixXf R); // Find out - how to update R?
      // f12 : get data from sensors (may have parameters according to sensors spec)
      //       Will set the measurement vector
      // VectorXf get_data();
      // f13 : get measurement
      void set_measurement(VectorXf z);
      // f14 : get measurement
      VectorXf get_measurement();
      // f15 : get time interval since last data read
      double get_time_interval();

      // these guys throw a warning. On the Hyped 2019 github code, they dont. Need to further invastigate why...
      static const float kStateCovarianceNoise = 0.01;
      static const float kMeasurementNoise = 0.01;

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
#endif  // KALMAN_FILTER_HPP
