#include <iostream>
#include <fstream>
#include <stdio.h>
#include <iomanip> 
#include "kalman_filter.hpp"
#include "IMUs_faker.hpp"

int main(){

  int n = 3;
  int m = 1;
  int k = 0;

  // MatrixXf A(n,n);
  // // A <<  1, 0.05, 0.00125, 0, 0  , 1, 0, 0  , 1,
  // //       0, 1  , 0.05, 0, 0  , 1, 0, 0  , 1,
  // //       0, 0  , 1, 0, 0  , 1, 0, 0  , 1,
  // //       0, 0  , 1, 0, 0  , 1, 0, 0  , 1,
  // //       0, 0  , 1, 0, 0  , 1, 0, 0  , 1,
  // //       0, 0  , 1, 0, 0  , 1, 0, 0  , 1,
  // //       0, 0  , 1, 0, 0  , 1, 0, 0  , 1,
  // //       0, 0  , 1, 0, 0  , 1, 0, 0  , 1,
  // //       0, 0  , 1, 0, 0  , 1, 0, 0  , 1;

  // MatrixXf R(m,m);
  // R <<  0.00441, 0.001231, 0.001543,
  //       0.0041, 0.0023, 0.005,
  //       0.00445, 0.005, 0.00521;

  // MatrixXf P(n,n);
  // P <<  0.0173073, 0.0104471, 0.0071627, 0.0076457, 0.0121524, 0.0065658, 0.0179352, 0.0108406, 0.0118031,
  //       0.0157204, 0.0128189, 0.0050831, 0.0049969, 0.0138789, 0.0140259, 0.0011413, 0.0132801, 0.0187289,
  //       0.0155873, 0.0020245, 0.0031738, 0.0122148, 0.0059844, 0.0111484, 0.0149733, 0.0132880, 0.0142017,
  //       0.0059790, 0.0033706, 0.0148544, 0.0191942, 0.0017601, 0.0086131, 0.0061238, 0.0186420, 0.0131628,
  //       0.0145752, 0.0060106, 0.0083761, 0.0003539, 0.0096337, 0.0123841, 0.0119311, 0.0045515, 0.0070880,
  //       0.0079593, 0.0146565, 0.0186382, 0.0191056, 0.0115639, 0.0120462, 0.0010751, 0.0092021, 0.0007501,
  //       0.0157149, 0.0099982, 0.0129054, 0.0106002, 0.0089878, 0.0178508, 0.0175514, 0.0146327, 0.0045925,
  //       0.0080400, 0.0150793, 0.0005705, 0.0080995, 0.0003788, 0.0028847, 0.0002806, 0.0155923, 0.0173318,
  //       0.0176278, 0.0033251, 0.0090343, 0.0199411, 0.0022355, 0.0008295, 0.0004930, 0.0051388, 0.0162699;

  // MatrixXf Q(n,n);
  // Q <<  0.0038199, 0.0021139, 0.0023765, 0.0040730, 0.0013697, 0.0017099, 0.0017959, 0.0014925, 0.0032818,
  //       0.0004785, 0.0008471, 0.0044741, 0.0032441, 0.0004911, 0.0035261, 0.0027967, 0.0015974, 0.0006194,
  //       0.0030674, 0.0043754, 0.0027404, 0.0048334, 0.0027530, 0.0044716, 0.0018135, 0.0012902, 0.0003169,
  //       0.0020580, 0.0015657, 0.0009940, 0.0044219, 0.0040990, 0.0046275, 0.0024295, 0.0041733, 0.0025956,
  //       0.0031883, 0.0043857, 0.0045614, 0.0047038, 0.0046152, 0.0010145, 0.0040409, 0.0048218, 0.0045871,
  //       0.0021975, 0.0002105, 0.0022208, 0.0029493, 0.0047779, 0.0039809, 0.0049728, 0.0042161, 0.0049217,
  //       0.0043559, 0.0001596, 0.0006298, 0.0040202, 0.0006812, 0.0014128, 0.0026376, 0.0037921, 0.0007287,
  //       0.0006844, 0.0045884, 0.0031292, 0.0038592, 0.0030004, 0.0030297, 0.0009226, 0.0049694, 0.0017948,
  //       0.0043098, 0.0019785, 0.0026877, 0.0034338, 0.0035733, 0.0044324, 0.0042381, 0.0019838, 0.0014801;

  // VectorXf s(n);
  // s << 0,0,0,0,0,0,0,0,0;

  // VectorXf z(m);

  MatrixXf A(n,n);
  A << 1, 0.05, 0.00125,
      0, 1  , 0.05,
      0, 0  , 1        ;

  MatrixXf R(m,m);
  R << 0.005;

  MatrixXf P(n,n);
  P << 0.333, 0.547665325, 0.5444,
      0.53, 0.41, 0.122,
      0.6, 0.3, 0.23;

  MatrixXf Q(n,n);
  Q << 0.021, 0.022, 0.0243,
      0.0242, 0.05322, 0.0214,
      0.0132, 0.021, 0.023;

  VectorXf s(n);
  s << 0,0,0;

  VectorXf z(m);

  KalmanFilter KF = KalmanFilter(n,m,k);
  KF.init(s, A, P, Q, R);

  KF.set_initial(s);

  std::cout << '\n';

  std::cout << "state: \n";

  demo_IMU_data IMU_demo = demo_IMU_data("state_acc.txt", m);
  vector< vector<float> > data = IMU_demo.get_data();
  float dt;

  VectorXf s_update(9);

  std::ofstream fout("text_results.txt");

  for(std::size_t i=0; i < data.size(); ++i)
  {

    for(int j = 0; j < m; j++)
    {
      z(j) = data[i][j+1];
    }

    if (i == 0){
      KF.filter(0.05,s,z);

      // KF.update_state_transition(0.05);

      // KF.predict_state();
      // KF.predict_covariance();
      // KF.estimate_state(KF.kalman_gain(), z);

      // std::cout << KF.get_state();
    }
    else{
      dt = (float)0.001*(data[i][0] - data[i-1][0]);

      std::cout << i;
      std::cout << " - data";
      std::cout << '\n';
      std::cout << '\n';
      std::cout << '\n';
      std::cout << '\n';

      KF.filter(dt,s,z);

      // std::cout << KF.x_;
      // std::cout << '\n';
      // std::cout << '\n';
    }

    // std::cout << KF.H_;

    // state update (Need to find out whether the state from the KF can update the speed and velocity)
    // I think that would work if the accceleration would be at the last entry of the vector which
    // cannot be done since from the current state formula x = x + K * (z - H * x) the result of K * (z - H * x)
    // would be (a,v,s) amd not (s,v,a). Some extreme testing has to be taken in order to either resolve,
    // this possible issue (it may cause less accuracy in printing the state by the vector below), but,
    // untill then, this solution can be considered.

    for(int j = 0; j < n; j++)
    {
      fout << KF.get_state()(0*m + j) << std::setw(15);

      // std::cout << KF.get_state()(0*m + j);

      if (j == n-1){
        fout << '\n';
        // std::cout << '\n';
      }
      else{
        // std::cout << ' ';
      }
    }
  }
  
  return 0;

}