#include <fstream>
#include <string>

#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "navigation/main.hpp"
#include "sensors/imu_manager.hpp"
#include "utils/concurrent/thread.hpp"

using hyped::utils::Logger;
using hyped::utils::System;
using hyped::utils::concurrent::Thread;

using hyped::data::Sensors;

int main(int argc, char* argv[])
{
  System::parseArgs(argc, argv);
  System& sys = System::getSystem();
  Logger log_system(sys.verbose, sys.debug);
  Logger log_nav(sys.verbose_nav, sys.debug_nav);
  Logger log_sensor(sys.verbose_sensor, sys.debug_sensor);

  // print HYPED logo at system startup
  std::ifstream file("main_logo.txt");
  if (file.is_open()) {
    std::string line;
    while (getline(file, line)) {
        printf("%s\n", line.c_str());
    }
    file.close();
  }

  log_system.INFO("MAIN", "Starting BBB with %d modules", 2);
  log_system.DBG("MAIN", "DBG0");
  log_system.DBG1("MAIN", "DBG1");
  log_system.DBG2("MAIN", "DBG2");
  log_system.DBG3("MAIN", "DBG3");

  // Initalise the threads here
  Thread* sensors = new hyped::sensors::ImuManager(log_sensor);
  Thread* nav     = new hyped::navigation::Main(1, log_nav);

  // Start the threads here
  sensors->start();
  nav->start();

  // Join the threads here
  sensors->join();
  nav->join();

  delete sensors;
  delete nav;

  return 0;
}