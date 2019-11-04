#include <iostream>
#include <vector>
#include <fstream>
#include "kalman_filter.cpp"

int main() {
  KalmanFilter kf(3, 1, 0);
  kf.set_initial(0.05);
  std::vector<float>measurements;
  std::ifstream fin("data.txt");
  int n;
  fin >> n;
  for(int i = 0; i < n; i++)
  {
    float ac;
    fin >> ac >> ac;
    measurements.push_back(ac);
  }
  fin.close();

  std::ofstream fout("predictions.txt");
  for(int i = 0; i < n; i++)
  {
    float measurement = measurements[i];
    VectorXf z = VectorXf::Constant(1, 0.0);
    z(0, 0) = measurement;
    kf.filter(z);
    // fout << kf.get_state() << "\n" << "\n";
    fout << kf.get_state()(0, 0) << " " << kf.get_state()(1, 0) << " " << kf.get_state()(2, 0) << "\n";
  }
  fout.close();
  return 0;
}
