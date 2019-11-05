#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP
#include <iostream>
#include <Eigen/Dense>
#include <vector>

//!!! General comment: fewer return statements, work more with member variables
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
    
      KalmanFilter(int n, int m, int k = 0);

      /*
      * Constructor call for a kalman filter
      * x : state
      * A : Transition Matrix
      * P : Covariance
      * Q : Random uncertainty covariacne
      * H : Measurement Matrix
      * R : Sensor Noise Covariance
      */
      void init(VectorXf x, MatrixXf A, MatrixXf P, MatrixXf Q, MatrixXf R);

      // set initial state and covariance
      void set_initial(VectorXf init);  //! Add initial covariance matrix

      // get state
      VectorXf get_state();

      // get covariacne
      MatrixXf get_covariance();

      // filter using 5 KF equations
      /*
      * dt : Time interval
      * s  : state
      * z  : sensor measurement vector
      */
      void filter(float dt, VectorXf s, VectorXf z);  //!! Two variables missing, update this! (real life values) - fixed

    public:
      // f1 : predict state
      void predict_state();
      // f2 : estimate state (update based on measurement)
      void estimate_state(MatrixXf K, VectorXf z);
      // f3 : kalman gain
      MatrixXf kalman_gain();
      // f4 : predict state covariance
      void predict_covariance();
      // f5 : estimate state covariance
      void estimate_covariance(MatrixXf K);
      // f10 : update state transition;
      void update_state_transition(float Dt);  //! lowercase d
      // f11 : update state transition;
      void update_sensor_noise(MatrixXf R); // Find out - how to update R?
      // f12 : get data from sensors (may have parameters according to sensors spec)
      //       Will set the measurement vector
      // Vector get_data();
      // f13 : get measurement
      void set_measurement(VectorXf z);
      // f14 : get measurement
      VectorXf get_measurement();
      // Setting Matrix H;
      MatrixXf set_measurement_matrix();



      int n_;
      int m_;
      int k_;

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
