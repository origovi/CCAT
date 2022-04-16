#include "utils/Time.hpp"

std::map<std::string, ros::Time> Time::clocks_;

void Time::tick(const std::string &clockName) {
  std::map<std::string, ros::Time>::iterator it = clocks_.find(clockName);
  if (it != clocks_.end()) {
    ROS_WARN("Called tick() two times with same clockName before calling tock()");
    it->second = ros::Time::now();
  } else {
    clocks_.emplace(clockName, ros::Time::now());
  }
}

void Time::tock(const std::string &clockName) {
  std::map<std::string, ros::Time>::iterator it = clocks_.find(clockName);
  if (it == clocks_.end()) {
    ROS_ERROR("Called tock() before calling tick()");
  }
  else {
    ROS_INFO_STREAM(it->first << " has taken: " << (ros::Time::now()-(it->second)).toSec()*1000 << "ms");
    clocks_.erase(it);
  }
}
