#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;

class demo_IMU_data{

    public:

        demo_IMU_data(string file_name, int n);

        vector< vector<float> > get_data();

    public:

        string  file_name_;
        int     m_;
};