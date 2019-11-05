#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;

class IMU_faker_data{

  public:

    IMU_faker_data(int m);

  private:

    IMU_faker_data(): m_(1){};

    int m_; // measurement dimentionality

};