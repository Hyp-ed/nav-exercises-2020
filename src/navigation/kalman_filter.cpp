#include "kalman_filter.hpp"

namespace hyped {
namespace navigation {

KalmanFilter::KalmanFilter(uint8_t n/*=3*/, uint8_t m/*=1*/, uint8_t k/*=0*/)
  : n_(n),
    m_(m),
    k_(k)
{}

}} // namespace