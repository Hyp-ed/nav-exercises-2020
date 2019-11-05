#include "IMUs_data_generator.cpp"

int main(){

  IMU_faker_data IMU = IMU_faker_data(1);

  IMU.generator(50, 0.1, 500, 7.9, 0.08, 0.03, 0.01, 0.08, 0.03);

}