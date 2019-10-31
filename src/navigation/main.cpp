#include <iostream>
#include "kalman_filter.cpp"

int main() {
  MatrixXf A(3, 3);
  A << 5, 2, 3, 4, 5, 6, 7, 8, 9;
  KalmanFilter aa(3, 3, 0);
  aa.set_initial(0.5);
  std::cout << aa.get_state() << std::endl;
  std::cout << A << std::endl;
  return 0;
}
