#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;

class IMU_faker_data{

  public:

    /*
    *  m : measurement dimentionality
    */

    IMU_faker_data(int m);

    /*
      * Constructor call for a kalman filter
      * dt            : dimension of the state
      * dt_noise      : noise of 
      * total         : Total fake measurements to be generated
      * avg_change    : Average change of the data per iteration (ex: -0.3, data is decrementing, +0.3 data is incrementing, 0.0 data is stable)
      * start_change  : Initial difference - for linear change, use avg_change
      * end_change    : Last difference - for linear change, use avg_change
      * change_noise  : Noise of change (noise assumes average change of noise. If non linear change of data, noise will also be non linear)
      */

    void generator(
      float dt, 
      float dt_noise, 
      int total,
      float first_value,
      float start_change, 
      float end_change, 
      float data_noise,
      float y_axis_noise,
      float z_axis_noise);

  private:

    IMU_faker_data(): m_(1){};

    int m_; // measurement dimentionality

};