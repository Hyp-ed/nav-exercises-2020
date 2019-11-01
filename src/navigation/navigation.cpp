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
           acceleration_(0, 0.)  // TODO(anyone): Make this so we have 3d acceleration!
{
  log_.INFO("NAV", "Navigation module started");
  for (unsigned int i = 0; i < data::Sensors::kNumImus; i++) {
    // filters_[i] = KalmanFilter(1, 1);
    // TODO(KF team) fix kalman filter setup
    // filters_[i].setup();
  }
  log_.INFO("NAV", "Navigation module initialised");
}

NavigationType Navigation::getAcceleration() const
{
  return acceleration_.value;
}

// TODO (anyone) IMU ALGORITHM, what happens when we ask for new values from sensors?
void Navigation::queryImus()
{
  NavigationVectorArray acc_raw;
  OnlineStatistics<NavigationType> acc_avg_filter;
  sensor_readings_ = data_.getSensorsImuData();
  uint32_t t = sensor_readings_.timestamp;
  // process raw values

  acceleration_.value = acc_avg_filter.getMean();
  acceleration_.timestamp = t;
}

void Navigation::updateData()
{
  data::Navigation nav_data;
  nav_data.acceleration               = getAcceleration();

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