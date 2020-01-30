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
  for(int k = 0; i < data::Sensors::kNumImus; k++) { //initialising all IMUs as reliable
    imu_reliable_[i] = true;
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
  sensor_readings_ = data_.getSensorsImuData();
  for(int i = 0; i < data::Sensors::kNumImus; i++) {
    for(int j = 0; j < 3; j++) {
      acc_raw[i][j] = sensor_readings_.value[i].acc[j];
    }
  }
  uint32_t t = sensor_readings_.timestamp;
  // Filtering
  float new_dt = float(t - acceleration_.timestamp) / 1e6;
  for(int i = 0; i < data::Sensors::kNumImus; i++) {
    for(int j = 0; j < 3; j++) {
      filters_[i * 3 + j].updateStateTransition(new_dt);
      VectorXf z = VectorXf::Constant(1, acc_raw[i][j]); // measurement vector
      filters_[i * 3 + j].filter(z);
      estimate_[j][i] = filters_[i * 3 + j].get_state()(0);
    }
  }
  // outlier detection
  for(int j = 0; j < 3; j++) {
    m_zscore(estimate_[j]);
  }

  // Arithmetic average of acceleration values
  for(int j = 0; j < 3; j++) {
    acceleration_.value[j] = 0;
    for(int i = 0; i < data::Sensors::kNumImus; i++) {
      acceleration_.value[j] += estimate_[j][i];
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
  if (counter_ % 1000 == 0) {  // kPrintFreq
    log_.DBG("NAV", "%d: Data Update: a_x=%.3f, a_y=%.3f, a_z=%.3f",
               counter_, acceleration_.value[0], acceleration_.value[1], acceleration_.value[2]);
  }
  counter_++;
}
void Navigation::navigate() {
  if (!is_init_) set_init();
  queryImus();
  updateData();
}
template <class OutlierType>
void Navigation::m_zscore(OutlierType& data_array) {
  OutlierType data_array_copy;
  const int length = data_array.size();
  for(int i = 0 ; i < length ; i++) {
    data_array_copy[i] = data_array[i];
  }
  int mid = length / 2;
  int unreliable_imus = 0;
  float median = 0;
  float mean   = 0;
  float medAD  = 0;
  float meanAD = 0;
  OutlierType medADarray;
  OutlierType meanADarray;
  OutlierType modZscore;

  for (int i = 0; i < length; i++) { // Detecting faulty IMUs
    if (data_array[i] == 0) {
      imu_reliable_[i] = false;
      unreliable_imus++;
    } else {
      imu_reliable_[i] = true;
    }
  }


  std::sort(std::begin(data_array_copy), std::end(data_array_copy));
  
  /* Check for 6 and 8 imus first
  int starting_index = 0;
  for(int i = 0; i < length; i++) {
    if (!(data_array_copy[i] == 0)) {
      starting_index = i;
    }
  }
  */

  if(unreliable_imus == data::Sensors::kNumImus / 2) {
    median
  }
  if (length % 2 == 0) {
    median = (data_array_copy[mid] + data_array_copy[mid - 1]) / 2;
  } else {
    median = data_array_copy[mid];
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
    medAD = (medADarray[mid] + medADarray[mid - 1]) / 2;
  } else {
    medAD = medADarray[mid];
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
      modZscore[i] = (data_arkNumImusray[i] - median) / (1.253314 * meanAD);
    }
  }
  for (int i = 0; i < length ; i++) {
    if(fabs(modZscore[i]) > 3.5) {
      data_array[i] = median;
    }
  }
}

void Navigation::set_init()
{
  prev_timestamp_ = utils::Timer::getTimeMicros();
  acceleration_.timestamp = prev_timestamp_;
  is_init_ = true;
}
}} // namespace hyped::navigation