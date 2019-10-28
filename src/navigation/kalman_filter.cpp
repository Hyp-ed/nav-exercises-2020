#include <iostream>
#include "kalman_filter.hpp"

// using namespace Eigen;
using Eigen::VectorXf;
using Eigen::MatrixXf;

KalmanFilter::KalmanFilter(uint8_t n, uint8_t m, uint8_t k)
    : n_(n)
    , m_(m)
    , k_(0){
}

void KalmanFilter::set_initial(VectorXf init){
    x_ =  init;
}

VectorXf KalmanFilter::get_state(){
    return x_;
}

MatrixXf KalmanFilter::get_covariance(){
    return P_;
}

double KalmanFilter::get_time_interval(){
    // TODO
    return 0.05; // Sample - get time measurement between now nad last calculation
}

VectorXf KalmanFilter::get_measurement(){

    return z_;
}

void KalmanFilter::update_sensor_noise(MatrixXf R){

    //some sort of calclulation takes place here.

    R_ = R;
}

void KalmanFilter::filter(){
    KalmanFilter::predict_state();
    KalmanFilter::predict_covariance();

    MatrixXf K = KalmanFilter::kalman_gain();

    // KalmanFilter::get_data(); // manipulate sensor data and set measurement vector

    KalmanFilter::estimate_state(K);
    KalmanFilter::estimate_covariance(K);

    double Dt = get_time_interval();
    KalmanFilter::update_state_transition(Dt);
}

VectorXf KalmanFilter::predict_state(){
    x_ = A_ * x_;
    return x_;
}

MatrixXf KalmanFilter::predict_covariance(){
    P_ = A_ * P_ * A_.transpose() + Q_;
    return P_;
}

MatrixXf KalmanFilter::kalman_gain(){
    MatrixXf K = (P_ * H_.transpose()) * (H_ * P_ * H_.transpose() + R_).inverse();
    return K;
}

VectorXf KalmanFilter::estimate_state(MatrixXf K){
    x_ = x_ + K * (z_ - H_ * x_);
    return x_;
}

MatrixXf KalmanFilter::estimate_covariance(MatrixXf K){
    P_ = (I_ - K * H_) * P_;
    return P_;
}

void KalmanFilter::update_state_transition(double Dt){
    MatrixXf A(n_,n_);

    A << 1.0, Dt, (Dt*Dt)/2,
        0.0, 1.0, Dt,
        0.0, 0.0, 1.0;

    A_ = A;
}

// VectorXf KalmanFilter::get_data(){

//     Eigen::Vector3f z(0.0,0.0,0.0); // placeholder

//     // read data from sensor
//     // manipulate data - make any calculations needed
//     // store data in a VectorXf format

//     // output data
//     set_measurement(z);

//     return z;
// }

void KalmanFilter::set_measurement(VectorXf z){
    z_ = z;
}




// #endif  // KALMAN_FILTER_HPP