#ifndef NAVIGATION_NAVIGATION_HPP_
#define NAVIGATION_NAVIGATION_HPP_
#include <array>
#include <cstdint>
#include <math.h>
#include "data/data.hpp"
#include "data/data_point.hpp"
#include "sensors/imu.hpp"
#include "navigation/kalman_filter.hpp"
#include "utils/logger.hpp"
#include "utils/math/integrator.hpp"
#include "utils/math/statistics.hpp"
namespace hyped {
using data::Data;
using data::DataPoint;
using data::ImuData;
using data::ModuleStatus;
using data::NavigationType;
using data::NavigationVector;
using navigation::KalmanFilter;
using utils::Logger;
using utils::math::OnlineStatistics;
using utils::math::RollingStatistics;
namespace navigation {
  class Navigation {
    public:
      typedef std::array<ImuData, data::Sensors::kNumImus>            ImuDataArray;
      typedef DataPoint<ImuDataArray>                                 ImuDataPointArray;
      typedef std::array<NavigationVector, data::Sensors::kNumImus>   NavigationVectorArray;
      typedef std::array<NavigationType, data::Sensors::kNumImus>     NavigationArray;
      typedef std::array<KalmanFilter, 3 * data::Sensors::kNumImus>       FilterArray;
      typedef std::array<data::StripeCounter, data::Sensors::kNumKeyence>  KeyenceDataArray;
      typedef std::array<NavigationArray, 3>  EstimateArray;
      /**
       * @brief Construct a new Navigation object
       *
       * @param log System logger
       */
      explicit Navigation(Logger& log);
      /**
       * @brief Get the measured acceleration [m/s^2]
       *
       * @return NavigationType Returns the forward component of acceleration vector (negative when
       *                        decelerating) [m/s^2]
       */
      NavigationVector getAcceleration() const;
      /**
       * @brief Update central data structure
       */
      void updateData();
      /**
       * @brief Take acceleration readings from IMU, filter, integrate and then update central data
       * structure with new values (i.e. the meat'n'potatoes of navigation).
       */
      void navigate();
    private:
      static constexpr int kPrintFreq = 1;
      // System communication
      Logger& log_;
      Data& data_;
      ModuleStatus status_;
      // counter for outputs
      unsigned int counter_;
      // Kalman filters to filter each IMU measurement individually
      FilterArray filters_;
      // To store estimated values
      ImuDataPointArray sensor_readings_;
      DataPoint<NavigationVector> acceleration_;
      EstimateArray estimate_;
      // To calculate estimates
      // previous timestamp
      uint32_t prev_timestamp_;
      // initial timestamp set
      bool is_init_;
      /**
       * @brief Query sensors to determine acceleration, velocity and distance
       */
      void queryImus();
      /**
       * @brief Set initial timestamps
       */
      void set_init();
      /**
       * @brief Detects outliers in data array based on modified z-score
       */
      template <class OutlierType>
      void m_zscore(OutlierType& data_array);
  };
}}  // namespace hyped::navigation
#endif  // NAVIGATION_NAVIGATION_HPP_