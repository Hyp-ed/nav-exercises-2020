#include <iostream>
#include "kalman_filter.hpp"
#include "IMUs_faker.hpp"
#include <fstream>
#include <stdio.h>

int main(){

  MatrixXf H(1,3);
  H << 1,0,0;

  MatrixXf I(3,3);
  I << 1,0,0,
      0,1,0,
      0,0,1;

  MatrixXf A(3,3);
  A << 1, 0.05, 0.00125,
      0, 1  , 0.05,
      0, 0  , 1        ;

  MatrixXf R(1,1);
  R << 0.005;

  MatrixXf P(3,3);
  P << 0.333, 0.547665325, 0.5444,
      0.53, 0.41, 0.122,
      0.6, 0.3, 0.23;

  MatrixXf Q(3,3);
  Q << 0.021, 0.022, 0.0243,
      0.0242, 0.05322, 0.0214,
      0.0132, 0.021, 0.023;

  VectorXf s(3);
  s << 0,0,0;

  VectorXf s_p(3);
  s_p << 0,0,0;

  VectorXf z(1);
  z << 7.3;


  KalmanFilter KF = KalmanFilter(3,1,0);
  KF.init(s, A, P, Q, H, R);

  KF.set_initial(s);

  std::cout << '\n';

  std::cout << "state: \n";

    // KF.update_state_transition(0.05);

    // KF.predict_state();
    // KF.predict_covariance();

    KF.update_state_transition(0.05);
    KF.predict_state();
    KF.predict_covariance();

     std::cout << KF.get_covariance();

    // MatrixXf K(n_, n_);
    // K = KF.kalman_gain();

    // // KF.get_data(); // manipulate sensor data and set measurement VectorXf

    // KF.estimate_state(K,z);

    // KF.estimate_covariance(K);

//   demo_IMU_data IMU_demo = demo_IMU_data("state_acc.txt");
//   vector< vector<float> > data = IMU_demo.get_data();
//   float dt;

//   VectorXf s_update(3);

//   std::ofstream fout("text_results.txt");
//   for(std::size_t i=0; i < data.size(); ++i)
//   {
//         z(1) = data[i][1];

//         if (i == 0){
//               KF.filter(0.05,s,z);
//         }
//         else{
//               dt = (float)0.001*(data[i][0] - data[i-1][0]);

//               KF.filter(dt,s,z);
//         }

//         // state update (Need to find out whether the state from the KF can update the speed and velocity)
//         // I think that would work if the accceleration would be at the last entry of the vector which
//         // cannot be done since from the current state formula x = x + K * (z - H * x) the result of K * (z - H * x)
//         // would be (a,v,s) amd not (s,v,a). Some extreme testing has to be taken in order to either resolve,
//         // this possible issue (it may cause less accuracy in printing the state by the vector below), but,
//         // untill then, this solution can be considered.
//         VectorXf s_update(3);
//         s_update << KF.get_state()(0), KF.get_state()(0) * dt + s_update(1), KF.get_state()(0) * dt * dt/2 + s_update(1) * dt + s_update(2);

//         fout << s_update;
//   }
  
  return 0;

}