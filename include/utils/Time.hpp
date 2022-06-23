/**
 * @file Time.hpp
 * @author Oriol Gorriz (oriol.gorriz@estudiantat.upc.edu)
 * @brief It contains the specification of the Time class.
 * @version 1.0
 * @date 2022-06-21
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 * 
 */

#ifndef UTILS_TIME_HPP
#define UTILS_TIME_HPP

#include <ros/ros.h>

#include <map>
#include <stdexcept>
#include <string>

/**
 * @brief Abstract-static class made to make it easier to quantify the time
 * a specific set of operation take.
 */
class Time {
 private:
  static std::map<std::string, ros::WallTime> clocks_;

 public:
  Time() = delete;
  static void tick(const std::string &clockName);
  static ros::WallDuration tock(const std::string &clockName);
};

#endif  // UTILS_TIME_HPP