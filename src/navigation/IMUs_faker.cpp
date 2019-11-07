#include "IMUs_faker.hpp"
#include <iostream>
#include <fstream>

using namespace std;
using Eigen::VectorXf;
using Eigen::MatrixXf;

demo_IMU_data::demo_IMU_data(string filename, int m)
{
    file_name_ = filename;
    m_         = m;
}

vector< vector<float> > demo_IMU_data::get_data()
{
    vector< vector<float> > data;

    string line;
    ifstream myfile (file_name_);
    int i = 0;

    if (myfile.is_open())
    {
        vector<float> dt_acc;

        dt_acc.push_back(0.0);
        for(int j = 0; j < m_; j++){
            dt_acc.push_back(0.0);
        }
        
        while (! myfile.eof() )
        {
            getline (myfile,line, '\t');
            dt_acc.at(0) = strtof((line).c_str(),0); //string to flaot

            for(int j = 0; j < m_; j++)
            {
                if (j == m_-1){
                    getline (myfile,line, '\n');
                }
                else{
                    getline (myfile,line, '\t');
                    // std::cout << line;
                }
                dt_acc.at(j+1) = strtof((line).c_str(),0); //string to flaot
            }

            data.push_back(dt_acc);

            //resetting touple
            dt_acc.at(0) = 0;

            for(int j = 0; j < m_; j++){
                dt_acc.at(j+1) = 0;
            }
                
        }
        myfile.close();
    }
    else cout << "Unable to open file";

    // vector<array<double, 2> > data = {{1.0,2.0},{3.0,4.0}};

    return data;
    
}