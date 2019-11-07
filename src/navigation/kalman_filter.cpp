#include <iostream>
#include "kalman_filter.hpp"
#include <vector>

// using namespace Eigen;
using namespace std;
using Eigen::VectorXf;
using Eigen::MatrixXf;

void KalmanFilter::init(VectorXf x, MatrixXf A, MatrixXf P, MatrixXf Q, MatrixXf R){
    x_ = x;
    A_ = A;
    P_ = P;
    Q_ = Q;
    H_ = set_measurement_matrix();
    R_ = R;
    I_ = MatrixXf::Identity(n_, n_);

    //creating the init MatrixXf. Find a way to add this to the MatrixXf_lib as a method and not allow the user to assign a new value to it.

    //creating the init MatrixXf. Find a way to add this to the MatrixXf_lib as a method and not allow the user to assign a new value to it.
}

KalmanFilter::KalmanFilter(int n, int m, int k)
    : n_(n)
    , m_(m)
    , k_(k){

    z_.resize(m);
    x_.resize(n);
    A_.resize(n,n);
    P_.resize(n,n);
    Q_.resize(n,n);
    R_.resize(m,m);
    I_.resize(n,n);
    H_.resize(m,n);
}

void KalmanFilter::set_initial(VectorXf init){
    x_ =  init;
}

MatrixXf KalmanFilter::set_measurement_matrix()
{
    MatrixXf H(m_, n_);
    H = MatrixXf::Zero(m_,n_);

    for(int i = 0; i < m_; i++)
    {
        H(i, n_ - m_ + i) = 1;
    }

    return H;
}

VectorXf KalmanFilter::get_state(){
    return x_;
}

MatrixXf KalmanFilter::get_covariance(){
    return P_;
}

VectorXf KalmanFilter::get_measurement(){
    return z_;
}

void KalmanFilter::update_sensor_noise(MatrixXf R){

    //some sort of calclulation take place here.

    R_ = R;
}

void KalmanFilter::filter(float dt, VectorXf s, VectorXf z){
    
    KalmanFilter::update_state_transition(dt);

    KalmanFilter::predict_state();

    KalmanFilter::predict_covariance();

    MatrixXf K(n_, n_);
    K = KalmanFilter::kalman_gain();

    // KalmanFilter::get_data(); // manipulate sensor data and set measurement VectorXf

    std::cout << x_;
    std::cout << '\n';
    std::cout << '\n';
    // std::cout << (z - H_ * x_);
    // std::cout << '\n';
    // std::cout << '\n';
    // std::cout << H_ * x_;
    // std::cout << '\n';
    // std::cout << '\n';
    // std::cout << z;
    // std::cout << '\n';
    // std::cout << '\n';
    // std::cout << K;
    // std::cout << '\n';
    // std::cout << '\n';
    // std::cout << P_;
    // std::cout << '\n';
    // std::cout << '\n';

    KalmanFilter::estimate_state(K,z);
    KalmanFilter::estimate_covariance(K);

}

void KalmanFilter::predict_state(){
    x_ = A_ * x_;
}

void KalmanFilter::predict_covariance(){
    P_ = A_ * P_ * A_.transpose() + Q_;
}

MatrixXf KalmanFilter::kalman_gain(){
    MatrixXf K(n_, m_);
    K = (P_ * H_.transpose()) * (H_ * P_ * H_.transpose() + R_).inverse();
    return K;
}

void KalmanFilter::estimate_state(MatrixXf K, VectorXf z){
    x_ = x_ + K * (z - H_ * x_);
}

void KalmanFilter::estimate_covariance(MatrixXf K){
    P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::update_state_transition(float dt){

    MatrixXf A(n_, n_);
    A = MatrixXf::Zero(n_,n_);

    VectorXf s (3);
    s << 1.0, dt, dt*dt/2; // {displacement, velocity, acceleration placeholders}

    for(int i = 0; i < n_; i++)
    {
        for(int j = i; j < n_; j++)
        {
            if ((j - i) % m_ == 0){
                A(i, j) = s((j - i)/m_);
            }
        }
    }
    A_ = A;
}

void KalmanFilter::set_measurement(VectorXf z){
    z_ = z;
}

// #endif  // KALMAN_FILTER_HPP