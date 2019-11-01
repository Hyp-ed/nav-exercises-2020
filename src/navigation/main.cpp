#include <iostream>

#include "navigation/main.hpp"

namespace hyped {
namespace navigation {

  Main::Main(uint8_t id, Logger& log)
    : Thread(id, log),
      log_(log),
      sys_(System::getSystem()),
      nav_(log) {}

  void Main::run()
  {
    log_.INFO("NAV", "Axis: %d", sys_.axis);
    log_.INFO("NAV", "Navigation waiting for calibration");

    // wait for calibration state for calibration
    while (sys_.running_) {
      nav_.navigate();
    }
  }
}}  // namespace hyped::navigation
