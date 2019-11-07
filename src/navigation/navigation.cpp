#include <algorithm>

#include "navigation/navigation.hpp"
#include "utils/concurrent/thread.hpp"
#include "utils/timer.hpp"

namespace hyped {

using hyped::utils::concurrent::Thread;

namespace navigation {

Navigation::Navigation(Logger& log)
         : log_(log),
           data_(Data::getInstance()),
           status_(ModuleStatus::kStart),
           counter_(0),
           acceleration_(0, NavigationVector())  // TODO(anyone): Make this so we have 3d acceleration!
{
  log_.INFO("NAV", "Navigation module started");
  for (unsigned int i = 0; i < data::Sensors::kNumImus; i++) {
    for(int j = 0; j < 3; j++) {
      filters_[i * 3 + j] = KalmanFilter(1, 1);
      // TODO(KF team) fix kalman filter setup
      filters_[i * 3 + j].set_initial(0); // initalising with dummy time interval(0), but can be changed later
    }
  }
  for(int i = 0; i < 3; i++) {
    acceleration_.value[i] = 0;
  }
  log_.INFO("NAV", "Navigation module initialised");
}

NavigationVector Navigation::getAcceleration() const
{
  return acceleration_.value;
}

// TODO (anyone) IMU ALGORITHM, what happens when we ask for new values from sensors?
void Navigation::queryImus()
{
  NavigationVectorArray acc_raw;
  OnlineStatistics<NavigationType> acc_avg_filter;
  sensor_readings_ = data_.getSensorsImuData();

  for(int i = 0; i < data::Sensors::kNumImus; i++) {
    for(int j = 0; j < 3; j++) {
      acc_raw[i][j] = sensor_readings_.value[i].acc[j];
    }
  }

  uint32_t t = sensor_readings_.timestamp;
  // process raw values


  float new_dt = float(t - acceleration_.timestamp) / 1e6;
  for(int i = 0; data::Sensors::kNumImus; i++) {
    for(int j = 0; j < 3; j++) {
      filters_[i * 3 + j].updateStateTransition(new_dt);
      VectorXf z = VectorXf::Constant(1, acc_raw[i][j]); // measurement vector
      filters_[i * 3 + j].filter(z);
      estimate_[i][j] = filters_[i * 3 + j].get_state()(0);
    }
  }

  // outlier detection missing!
  // outlier_detec(estimate_);
  
  for(int j = 0; j < 3; j++) {
    acceleration_.value[j] = 0;
    for(int i = 0; i < data::Sensors::kNumImus; i++) {
      acceleration_.value[j] += estimate_[i][j];
    }
    acceleration_.value[j] /= (float) data::Sensors::kNumImus; 
  }
  acceleration_.timestamp = t;
}

void Navigation::updateData()
{
  data::Navigation nav_data;
  nav_data.acceleration               = getAcceleration()[0];

  data_.setNavigationData(nav_data);

  if (counter_ % 100 == 0) {  // kPrintFreq
    log_.DBG("NAV", "%d: Data Update: a_x=%.3f", // TODO(anyone) here data will be printed!
               counter_, nav_data.acceleration);
  }
  counter_++;
}

void Navigation::navigate()
{
  queryImus();
  updateData();
}
}}  // namespace hyped::navigation