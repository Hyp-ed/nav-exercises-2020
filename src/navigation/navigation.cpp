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

void Navigation::navigate() {
  if (!is_init_) set_init();
  queryImus();
  updateData();
}

void Navigation::m_zscore(NavigationArray& data_array) {

  NavigationArray data_array_copy;
  const int length = data_array.size(); 

  for(int i = 0 ; i < length ; i++) {      
    data_array_copy[i] = data_array[i];
  } 

  int mid = (length / 2) - 1;
  float median = 0;
  float mean   = 0;
  float medAD  = 0;
  float meanAD = 0;
  NavigationArray medADarray;
  NavigationArray meanADarray;
  NavigationArray modZscore;

  std::sort(std::begin(data_array_copy), std::end(data_array_copy));

  if (length % 2 == 0) {
    median = (data_array_copy[mid] + data_array_copy[mid + 1]) / 2;
  } else {
    median = data_array_copy[mid + 1];
  }

  for (int i = 0; i < length; i++) {
    mean += data_array_copy[i];
  }
  mean = mean / length;

  for (int i = 0; i < length; i++) {
    medADarray[i] = fabs(data_array[i] - median);  
  }

  std::sort(std::begin(medADarray), std::end(medADarray));
  
  if (length % 2 == 0) {
    medAD = (medADarray[mid] + medADarray[mid + 1]) / 2; 
  } else {
    medAD = medADarray[mid + 1];
  }
  
  for (int i = 0; i < length; i++) {
    meanADarray[i] = fabs(data_array[i] - mean); 
  }

  for (int i = 0; i < length; i++) {
    meanAD += meanADarray[i];
  }
  meanAD = meanAD / length;
  
  for (int i = 0; i < length; i++) {
    if (medAD != 0) {
      modZscore[i] = (data_array[i] - median) / (1.486 * medAD);
    } else {
      modZscore[i] = (data_array[i] - median) / (1.253314 * meanAD); 
    }
  }

  for (int i = 0; i < length ; i++) {
    if(fabs(modZscore[i]) > 3.5) {
      data_array[i] = median;
    }
  }
  
  void Navigation::set_init()
{
  prev_timestamp_ = utils::Timer::getTimeMicros();
  is_init_ = true;
}

}}} // namespace hyped::navigation