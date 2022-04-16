#ifndef UTILS_TIME_HPP
#define UTILS_TIME_HPP

#include <ros/ros.h>

#include <map>
#include <stdexcept>
#include <string>

class Time {
 private:
  static std::map<std::string, ros::Time> clocks_;

 public:
  Time() = delete;
  static void tick(const std::string &clockName);
  static void tock(const std::string &clockName);
};

#endif  // UTILS_TIME_HPP