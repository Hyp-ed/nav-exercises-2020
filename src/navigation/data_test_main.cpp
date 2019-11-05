#include "IMUs_faker.hpp"

int main(){

  demo_IMU_data IMU_demo = demo_IMU_data("state_acc.txt");
  vector< vector<float> > data = IMU_demo.get_data();

  int interval_print;

  for(std::size_t i=0; i < data.size(); ++i)
  {

    if (interval_print == 1)
    {
      std::cout << data[i-1][1] - data[i][1];
      std::cout << '\n';
      interval_print = 0;
    };
    interval_print++;

  }
}