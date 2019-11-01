#ifndef NAVIGATION_MAIN_HPP_
#define NAVIGATION_MAIN_HPP_

#include "data/data.hpp"
#include "navigation/navigation.hpp"
#include "utils/concurrent/thread.hpp"
#include "utils/system.hpp"
#include "utils/logger.hpp"

namespace hyped {

using data::Data;
using data::State;
using hyped::data::StateMachine;
using hyped::data::ModuleStatus;
using utils::concurrent::Thread;
using utils::System;
using utils::Logger;

namespace navigation {

class Main: public Thread {
  public:
    explicit Main(uint8_t id, Logger& log);
    void run() override;
    bool isCalibrated();
  private:
    Logger& log_;
    System& sys_;
    Navigation nav_;
};

}}  // namespace hyped::navigation

#endif  // NAVIGATION_MAIN_HPP_