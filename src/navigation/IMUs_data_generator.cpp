#include <iostream>
#include <fstream>
#include <stdio.h>
#include "IMUs_data_generator.hpp"

IMU_faker_data::IMU_faker_data(int m):
  m_(m)
{

};

void IMU_faker_data::generator(float dt,
                              float dt_noise, 
                              int total,
                              float first_value,
                              float start_change, 
                              float end_change, 
                              float data_noise,
                              float y_axis_noise,
                              float z_axis_noise)
{

  srand(time(0));

  double data_noise_rand;
  double time_noise_rand;
  double y_noise_rand;
  double z_noise_rand;

  float t   = 0.0;
  float acc = first_value;
  float acc_y = 0;
  float acc_z = 0;

  std::ofstream fout("fake_state_acc.txt");

  // for y and z access only add noise to 0 acceleration

  for(int i = 0; i < total; i++)
  {

    data_noise_rand = (2*((double) rand() / (RAND_MAX)) - 1) * data_noise;
    time_noise_rand = (2*((double) rand() / (RAND_MAX)) - 1) * dt_noise;
    y_noise_rand = (2*((double) rand() / (RAND_MAX)) - 1) * y_axis_noise;
    z_noise_rand = (2*((double) rand() / (RAND_MAX)) - 1) * z_axis_noise;

    acc   = acc - ((start_change - end_change)/(i+1)*20) + data_noise_rand;
    // acc_y = acc_y - y_axis_noise;
    // acc_z = acc_z - z_axis_noise;

    fout << t;
    fout << '\t';
    fout << acc;
    fout << '\t';
    fout << y_noise_rand;
    fout << '\t';
    fout << z_noise_rand;
    fout << '\n';

    t = t + dt + time_noise_rand;
  };

}